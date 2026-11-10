#!/usr/bin/env python3
"""
Laboratorio 2 - Microcontroladores
Monitor gráfico del control térmico con ESP32.

Interfaz (Tkinter + Matplotlib, sin dependencias extra de interfaz):
  - Selector de puerto con refresco y conexión/desconexión con un botón.
  - Tarjetas con temperatura, setpoint, modo, duty de lámparas y de cooler.
  - Aviso visible de alarma (falla de sensor o sobretemperatura).
  - Dos gráficas sincronizadas: temperatura/setpoint y ciclos de trabajo.
  - Ventana de tiempo configurable y botón de pausa de la gráfica.
  - Métricas del control automático (subida, sobreimpulso, establecimiento...).
  - Exportación a Excel (datos + métricas) y a figura PDF/PNG para el informe.
  - Modo demostración sin hardware, útil para probar la interfaz.

El CSV de cada sesión se guarda de forma continua en la carpeta "registros"
(junto a este script), por lo que no se pierden datos si algo falla.

Formato esperado desde el ESP32 (115200 baudios, 4 Hz):
  Tiempo_ms,Temperatura_C,Setpoint_C,Duty_Cooler,Duty_SSR,Modo,Alarma

Uso:
  python monitor_gui.py
  python monitor_gui.py --demo      # arranca en modo demostración

Cerrar el Monitor Serie de PlatformIO antes de conectar.
"""

import argparse
import csv
import os
import queue
import random
import subprocess
import sys
import threading
import time
from bisect import bisect_left
from datetime import datetime

import tkinter as tk
from tkinter import messagebox, ttk

import pandas as pd
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# pyserial es opcional: sin él, la interfaz funciona en modo demostración.
try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover
    serial = None
    list_ports = None

# ============================================================================
#  CONFIGURACIÓN
# ============================================================================
BAUDIOS = 115200
RANGO_PROPORCIONAL_C = 20.0       # Debe coincidir con RANGO_PROPORCIONAL del firmware
TEMP_CORTE_C = 110.0              # Debe coincidir con TEMP_CORTE_C del firmware
BANDA_ESTABLECIMIENTO_C = 2.0     # Banda ±2 °C para el tiempo de establecimiento
PERIODO_REFRESCO_S = 0.25         # Redibujo de la gráfica a 4 Hz como máximo
SIN_DATOS_AVISO_S = 3.0           # Aviso si no llegan datos durante este tiempo

CARPETA_REGISTROS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "registros")

COLUMNAS = ["Tiempo (s)", "Temperatura (°C)", "Setpoint (°C)",
            "Duty Cooler (%)", "Duty SSR (%)", "Modo", "Alarma"]

VENTANAS = {"30 s": 30, "1 min": 60, "2 min": 120, "5 min": 300, "Toda la sesión": None}

# Paleta (tema oscuro)
BG = "#0f172a"
PANEL = "#1e293b"
PANEL2 = "#273449"
BORDE = "#334155"
TEXTO = "#e2e8f0"
APAGADO = "#94a3b8"
ROJO = "#f87171"
NARANJA = "#fb923c"
VERDE = "#4ade80"
AZUL = "#60a5fa"
CIAN = "#22d3ee"
MORADO = "#c084fc"
AMBAR = "#fbbf24"

F_TITULO = ("Segoe UI", 9, "bold")
F_VALOR = ("Segoe UI", 30, "bold")
F_DETALLE = ("Segoe UI", 10)
F_NORMAL = ("Segoe UI", 10)
F_NEGRITA = ("Segoe UI", 10, "bold")
F_MONO = ("Consolas", 10)


# ============================================================================
#  LÓGICA SIN INTERFAZ (se puede probar por separado)
# ============================================================================
def parsear_linea(linea):
    """
    Convierte una línea CSV del ESP32 en una tupla
    (t_ms, temp, sp, duty_cooler, duty_ssr, modo, alarma).
    Devuelve None si es la cabecera, está vacía o está corrupta.
    """
    if not linea or linea.startswith("Tiempo_ms"):
        return None
    campos = linea.split(",")
    if len(campos) != 7:
        return None
    try:
        return (float(campos[0]), float(campos[1]), float(campos[2]),
                int(campos[3]), int(campos[4]), int(campos[5]), int(campos[6]))
    except ValueError:
        return None


def control_auto(temp, sp):
    """Replica las dos rectas del firmware (con duty mínimo 0). Devuelve (ssr, fan)."""
    lampara_off = sp - 1.0
    lampara_max = sp - RANGO_PROPORCIONAL_C
    if temp >= lampara_off:
        ssr = 0
    elif temp <= lampara_max:
        ssr = 100
    else:
        ssr = round((lampara_off - temp) / (lampara_off - lampara_max) * 100)

    cooler_max = sp + RANGO_PROPORCIONAL_C
    if temp <= sp:
        fan = 0
    elif temp >= cooler_max:
        fan = 100
    else:
        fan = round((temp - sp) / (cooler_max - sp) * 100)
    return ssr, fan


def limites_temperatura(temps, sps):
    """Límites del eje de temperatura con margen superior para la leyenda."""
    lo = min(min(temps), min(sps))
    hi = max(max(temps), max(sps))
    margen = max(hi - lo, 5.0)
    return lo - 0.06 * margen - 1, hi + 0.30 * margen + 1


def tramos(df, condicion):
    """Lista de (t_inicio, t_fin) donde la condición booleana es verdadera."""
    t = df["Tiempo (s)"].to_numpy()
    c = condicion.to_numpy()
    salida, ini = [], None
    for i in range(len(c)):
        if c[i] and ini is None:
            ini = t[i]
        if ini is not None and (not c[i] or i == len(c) - 1):
            salida.append((ini, t[i]))
            ini = None
    return salida


def calcular_metricas(df):
    """
    Métricas sobre el último tramo continuo en modo automático (sin alarma)
    con setpoint constante. Devuelve un diccionario o None si no hay datos.
    """
    valido = (df["Modo"] == 1) & (df["Alarma"] == 0)
    if not valido.any():
        return None

    grupo = (~valido).cumsum()
    seg = df[valido & (grupo == grupo[valido].iloc[-1])]

    sp = seg["Setpoint (°C)"].iloc[-1]
    cambios = seg[seg["Setpoint (°C)"] != sp]
    if not cambios.empty:
        seg = seg[seg.index > cambios.index[-1]]
    if len(seg) < 10:
        return None

    t = seg["Tiempo (s)"].to_numpy()
    T = seg["Temperatura (°C)"].to_numpy()
    t0, T0 = t[0], T[0]
    salto = sp - T0

    m = {"Setpoint (°C)": sp,
         "Temperatura inicial (°C)": T0,
         "Duración del tramo (s)": t[-1] - t0}

    if salto > 0:
        t10 = next((ti for ti, Ti in zip(t, T) if Ti >= T0 + 0.1 * salto), None)
        t90 = next((ti for ti, Ti in zip(t, T) if Ti >= T0 + 0.9 * salto), None)
        m["Tiempo de subida 10-90 % (s)"] = (t90 - t10) if (t10 is not None and t90 is not None) else None
        m["Sobreimpulso (%)"] = max(0.0, (T.max() - sp) / salto * 100.0)
    else:
        m["Tiempo de subida 10-90 % (s)"] = None
        m["Sobreimpulso (%)"] = None
    m["Temperatura máxima (°C)"] = float(T.max())

    fuera = [ti for ti, Ti in zip(t, T) if abs(Ti - sp) > BANDA_ESTABLECIMIENTO_C]
    if not fuera:
        m["Tiempo de establecimiento (s)"] = 0.0
    elif fuera[-1] >= t[-1] - 1e-9:
        m["Tiempo de establecimiento (s)"] = None      # No llegó a establecerse
    else:
        m["Tiempo de establecimiento (s)"] = fuera[-1] - t0

    n = max(3, len(T) // 5)
    m["Error estado estacionario (°C)"] = float(T[-n:].mean() - sp)
    m["Oscilación pico a pico final (°C)"] = float(T[-n:].max() - T[-n:].min())
    return m


def formatear_metricas(m):
    """Texto alineado para mostrar las métricas en el panel."""
    if m is None:
        return ("Sin datos suficientes.\n\n"
                "Se necesita un tramo en modo AUTOMÁTICO de al menos 10 muestras, "
                "sin alarma y con el setpoint sin cambiar.")
    ancho = max(len(k) for k in m)
    lineas = []
    for k, v in m.items():
        valor = "N/A" if v is None else f"{v:.2f}"
        lineas.append(f"{k.ljust(ancho)}  {valor}")
    return "\n".join(lineas)


def figura_sesion(df):
    """Figura con toda la sesión para el informe (fondo claro, apta para imprimir)."""
    fig = Figure(figsize=(10, 7), dpi=100)
    ax1, ax2 = fig.subplots(2, 1, sharex=True, gridspec_kw={"height_ratios": [3, 2]})
    t = df["Tiempo (s)"]

    ax1.plot(t, df["Temperatura (°C)"], color="tab:red", label="Temperatura")
    ax1.plot(t, df["Setpoint (°C)"], color="tab:orange", linestyle="--", label="Setpoint")
    ax1.set_ylabel("Temperatura (°C)")
    ax1.set_title("Control térmico ESP32 - sesión completa")
    ax1.grid(True, linestyle="--", alpha=0.5)

    ax2.plot(t, df["Duty SSR (%)"], color="tab:purple", label="Duty lámparas (SSR)")
    ax2.plot(t, df["Duty Cooler (%)"], color="tab:cyan", label="Duty cooler")
    ax2.set_ylabel("Duty cycle (%)")
    ax2.set_xlabel("Tiempo (s)")
    ax2.set_ylim(-5, 135)
    ax2.set_yticks([0, 25, 50, 75, 100])
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax1.set_ylim(*limites_temperatura(df["Temperatura (°C)"], df["Setpoint (°C)"]))

    manual = tramos(df, df["Modo"] == 0)
    for i, (a, b) in enumerate(manual):
        for ax in (ax1, ax2):
            ax.axvspan(a, b, color="gray", alpha=0.15,
                       label="Modo manual" if (i == 0 and ax is ax1) else None)
    alarmas = tramos(df, df["Alarma"] == 1)
    for i, (a, b) in enumerate(alarmas):
        for ax in (ax1, ax2):
            ax.axvspan(a, b, color="red", alpha=0.15,
                       label="Alarma" if (i == 0 and ax is ax1) else None)

    ax1.legend(loc="upper left", ncol=2)
    ax2.legend(loc="upper left", ncol=2)
    fig.tight_layout()
    return fig


def exportar_sesion(df, base, metricas):
    """Guarda Excel (Datos + Métricas), figura PDF y PNG. Devuelve la lista de rutas."""
    ruta_xlsx = f"{base}.xlsx"
    with pd.ExcelWriter(ruta_xlsx, engine="openpyxl") as escritor:
        df.to_excel(escritor, sheet_name="Datos", index=False)
        if metricas:
            pd.DataFrame(list(metricas.items()), columns=["Métrica", "Valor"]) \
              .to_excel(escritor, sheet_name="Métricas", index=False)

    fig = figura_sesion(df)
    fig.savefig(f"{base}.pdf")
    fig.savefig(f"{base}.png", dpi=200)
    return [ruta_xlsx, f"{base}.pdf", f"{base}.png"]


def abrir_carpeta(ruta):
    """Abre una carpeta en el explorador del sistema operativo."""
    try:
        if sys.platform.startswith("win"):
            os.startfile(ruta)                       # type: ignore[attr-defined]
        elif sys.platform == "darwin":
            subprocess.Popen(["open", ruta])
        else:
            subprocess.Popen(["xdg-open", ruta])
    except Exception as e:                           # pragma: no cover
        messagebox.showerror("No se pudo abrir la carpeta", str(e))


def detectar_puertos():
    """Devuelve {etiqueta: dispositivo} y la etiqueta sugerida (o None)."""
    if list_ports is None:
        return {}, None
    claves = ("cp210", "ch340", "ch910", "usb", "uart", "silicon labs")
    puertos, sugerido = {}, None
    for p in list_ports.comports():
        etiqueta = f"{p.device} — {p.description}"
        puertos[etiqueta] = p.device
        if sugerido is None and any(k in f"{p.description} {p.manufacturer or ''}".lower() for k in claves):
            sugerido = etiqueta
    return puertos, sugerido


# ============================================================================
#  FUENTES DE DATOS (hilos que alimentan una cola)
# ============================================================================
class FuenteSerial(threading.Thread):
    """Lee el puerto serie y publica tuplas en la cola. Nunca toca la interfaz."""

    def __init__(self, puerto, cola):
        super().__init__(daemon=True)
        self.puerto = puerto
        self.cola = cola
        self._parar = threading.Event()

    def detener(self):
        self._parar.set()

    def run(self):
        try:
            ser = serial.Serial(self.puerto, BAUDIOS, timeout=0.3)
        except Exception as e:
            self.cola.put(("error", f"No se pudo abrir {self.puerto}.\n\n{e}\n\n"
                                    "Cierra el Monitor Serie de PlatformIO y verifica el puerto."))
            return

        t_inicial, t_prev_ms, t_rel_s = None, None, 0.0
        try:
            ser.reset_input_buffer()
            while not self._parar.is_set():
                try:
                    bruto = ser.readline()           # Bloquea hasta 0,3 s: no consume CPU
                except Exception as e:
                    self.cola.put(("error", f"Se perdió la comunicación serial.\n\n{e}"))
                    return
                # errors="ignore": un byte corrupto no debe matar el hilo
                datos = parsear_linea(bruto.decode("utf-8", errors="ignore").strip())
                if datos is None:
                    continue
                t_ms, temp, sp, d_cooler, d_ssr, modo, alarma = datos
                if t_inicial is None:
                    t_inicial = t_ms                 # El tiempo de la sesión parte de 0
                elif t_ms < t_prev_ms:
                    # El ESP32 se reinició (millis() vuelve a 0): se continúa la línea de tiempo
                    t_inicial = t_ms - t_rel_s * 1000.0
                t_prev_ms = t_ms
                t_rel_s = (t_ms - t_inicial) / 1000.0
                self.cola.put(("dato", (t_rel_s, temp, sp, d_cooler, d_ssr, modo, alarma)))
        finally:
            ser.close()


class FuenteDemo(threading.Thread):
    """
    Simulador térmico (primer orden + control proporcional) para probar la
    interfaz sin hardware. Cada muestra simula 1 s y se emite a 4 Hz (x4).
    """

    def __init__(self, cola):
        super().__init__(daemon=True)
        self.cola = cola
        self._parar = threading.Event()
        self.rng = random.Random(7)

    def detener(self):
        self._parar.set()

    def run(self):
        t, T, ambiente, sp, dt = 0.0, 22.0, 22.0, 60.0, 1.0
        while not self._parar.is_set():
            if t >= 360:
                sp = 70.0
            if t >= 700:
                sp = 55.0

            modo = 0 if t < 20 else 1                 # Primeros 20 s en manual
            ssr, fan = control_auto(T, sp) if modo else (0, 0)

            calor = 0.25 * ssr / 100.0                # °C/s aportados por las lámparas
            perdida = 0.0008 * (T - ambiente)          # Pérdidas al ambiente
            enfriamiento = 0.012 * fan / 100.0 * (T - ambiente)
            T += (calor - perdida - enfriamiento) * dt + self.rng.gauss(0, 0.04)

            medida = round((T + self.rng.gauss(0, 0.1)) * 4) / 4   # Resolución 0,25 °C
            self.cola.put(("dato", (t, medida, sp, fan, ssr, modo, 0)))
            t += dt
            self._parar.wait(0.25)


# ============================================================================
#  COMPONENTES DE INTERFAZ
# ============================================================================
class Tarjeta(tk.Frame):
    """Tarjeta con título, valor grande, detalle y barra de progreso opcional."""

    def __init__(self, master, titulo, estilo_barra=None):
        super().__init__(master, bg=PANEL, padx=16, pady=12,
                         highlightthickness=1, highlightbackground=BORDE)
        tk.Label(self, text=titulo.upper(), bg=PANEL, fg=APAGADO,
                 font=F_TITULO).pack(anchor="w")
        self.valor = tk.Label(self, text="--", bg=PANEL, fg=TEXTO, font=F_VALOR)
        self.valor.pack(anchor="w")
        self.detalle = tk.Label(self, text=" ", bg=PANEL, fg=APAGADO, font=F_DETALLE)
        self.detalle.pack(anchor="w")
        self.barra = None
        if estilo_barra:
            self.barra = ttk.Progressbar(self, style=estilo_barra, maximum=100, length=160)
            self.barra.pack(fill="x", pady=(8, 0))

    def poner(self, valor, detalle=" ", color=TEXTO, barra=None):
        self.valor.config(text=valor, fg=color)
        self.detalle.config(text=detalle)
        if self.barra is not None and barra is not None:
            self.barra["value"] = barra


def boton(master, texto, comando, color=AZUL, ancho=None):
    """Botón plano con colores del tema."""
    fg = TEXTO if color == PANEL2 else BG
    return tk.Button(master, text=texto, command=comando, bg=color, fg=fg,
                     activebackground=TEXTO, activeforeground=BG, relief="flat",
                     font=F_NEGRITA, padx=12, pady=6, cursor="hand2", bd=0,
                     width=ancho or 0)


# ============================================================================
#  APLICACIÓN
# ============================================================================
class App(tk.Tk):
    def __init__(self, demo=False):
        super().__init__()
        self.title("Monitor de control térmico · ESP32")
        self.configure(bg=BG)
        self.geometry("1320x820")
        self.minsize(1100, 700)
        self.protocol("WM_DELETE_WINDOW", self._cerrar)

        self.fuente = None
        self.cola = queue.Queue()
        self.conectado = False
        self.puertos = {}
        self.demo_var = tk.BooleanVar(value=demo or serial is None)
        self.pausa_var = tk.BooleanVar(value=False)
        self.ventana_var = tk.StringVar(value="2 min")
        self.puerto_var = tk.StringVar()

        self._reiniciar_datos()
        self._configurar_estilos()

        self.columnconfigure(0, weight=1)
        self.rowconfigure(3, weight=1)
        self._construir_barra()
        self._construir_alarma()
        self._construir_tarjetas()
        self._construir_principal()
        self._construir_estado()

        self._actualizar_puertos()
        self._alternar_demo()
        self.after(100, self._sondear)

    # ------------------------------------------------------------------ estado
    def _reiniciar_datos(self):
        self.t, self.T, self.sp = [], [], []
        self.fan, self.ssr, self.modo, self.alarma = [], [], [], []
        self.t_min, self.t_max = None, None
        self.ultimo_dibujo = 0.0
        self.t_ultimo_dato = None
        self.t_conexion = None
        self.exportado = True
        self.ruta_csv = None
        self.archivo_csv = None
        self.escritor_csv = None

    # ------------------------------------------------------------------ estilos
    def _configurar_estilos(self):
        estilo = ttk.Style(self)
        estilo.theme_use("clam")
        estilo.configure("TCombobox", fieldbackground=PANEL2, background=PANEL2,
                         foreground=TEXTO, arrowcolor=TEXTO, bordercolor=BORDE,
                         lightcolor=PANEL2, darkcolor=PANEL2, selectbackground=PANEL2,
                         selectforeground=TEXTO)
        estilo.map("TCombobox", fieldbackground=[("readonly", PANEL2)],
                   foreground=[("readonly", TEXTO)])
        for nombre, color in (("Lampara", MORADO), ("Cooler", CIAN)):
            estilo.configure(f"{nombre}.Horizontal.TProgressbar", troughcolor=BG,
                             background=color, bordercolor=BORDE,
                             lightcolor=color, darkcolor=color, thickness=10)
        self.option_add("*TCombobox*Listbox.background", PANEL2)
        self.option_add("*TCombobox*Listbox.foreground", TEXTO)
        self.option_add("*TCombobox*Listbox.selectBackground", AZUL)
        self.option_add("*TCombobox*Listbox.selectForeground", BG)

    # ------------------------------------------------------------------ barra superior
    def _construir_barra(self):
        barra = tk.Frame(self, bg=PANEL, padx=14, pady=10)
        barra.grid(row=0, column=0, sticky="ew")

        tk.Label(barra, text="🌡  Control térmico ESP32", bg=PANEL, fg=TEXTO,
                 font=("Segoe UI", 13, "bold")).pack(side="left", padx=(0, 24))

        tk.Label(barra, text="Puerto", bg=PANEL, fg=APAGADO, font=F_NORMAL).pack(side="left")
        self.combo_puerto = ttk.Combobox(barra, textvariable=self.puerto_var,
                                         state="readonly", width=38)
        self.combo_puerto.pack(side="left", padx=(6, 4))
        self.btn_refrescar = boton(barra, "⟳", self._actualizar_puertos, PANEL2, 3)
        self.btn_refrescar.pack(side="left", padx=(0, 14))

        self.chk_demo = tk.Checkbutton(barra, text="Modo demostración (sin hardware)",
                                       variable=self.demo_var, command=self._alternar_demo,
                                       bg=PANEL, fg=TEXTO, selectcolor=PANEL2,
                                       activebackground=PANEL, activeforeground=TEXTO,
                                       font=F_NORMAL)
        self.chk_demo.pack(side="left", padx=(0, 14))

        self.btn_conectar = boton(barra, "Conectar", self._alternar_conexion, VERDE, 14)
        self.btn_conectar.pack(side="left")

        self.lbl_estado = tk.Label(barra, text="● Desconectado", bg=PANEL, fg=APAGADO,
                                   font=F_NEGRITA)
        self.lbl_estado.pack(side="right")

    # ------------------------------------------------------------------ banner de alarma
    def _construir_alarma(self):
        self.banner = tk.Label(self, text="", bg=ROJO, fg=BG,
                               font=("Segoe UI", 12, "bold"), pady=8)
        self.banner.grid(row=1, column=0, sticky="ew")
        self.banner.grid_remove()

    # ------------------------------------------------------------------ tarjetas
    def _construir_tarjetas(self):
        marco = tk.Frame(self, bg=BG, padx=14, pady=12)
        marco.grid(row=2, column=0, sticky="ew")
        for i in range(5):
            marco.columnconfigure(i, weight=1, uniform="tarjeta")

        self.c_temp = Tarjeta(marco, "Temperatura")
        self.c_sp = Tarjeta(marco, "Setpoint")
        self.c_modo = Tarjeta(marco, "Modo de operación")
        self.c_ssr = Tarjeta(marco, "Lámparas (SSR)", "Lampara.Horizontal.TProgressbar")
        self.c_fan = Tarjeta(marco, "Cooler (PWM)", "Cooler.Horizontal.TProgressbar")
        for i, c in enumerate((self.c_temp, self.c_sp, self.c_modo, self.c_ssr, self.c_fan)):
            c.grid(row=0, column=i, sticky="nsew", padx=6)
        self._limpiar_tarjetas()

    def _limpiar_tarjetas(self):
        self.c_temp.poner("-- °C", "Esperando datos...")
        self.c_sp.poner("-- °C", " ")
        self.c_modo.poner("--", " ")
        self.c_ssr.poner("-- %", " ", barra=0)
        self.c_fan.poner("-- %", " ", barra=0)

    # ------------------------------------------------------------------ zona principal
    def _construir_principal(self):
        principal = tk.Frame(self, bg=BG, padx=14)
        principal.grid(row=3, column=0, sticky="nsew")
        principal.columnconfigure(0, weight=1)
        principal.rowconfigure(0, weight=1)

        # --- Gráficas ---
        izq = tk.Frame(principal, bg=PANEL, highlightthickness=1, highlightbackground=BORDE)
        izq.grid(row=0, column=0, sticky="nsew", padx=(6, 8))
        izq.columnconfigure(0, weight=1)
        izq.rowconfigure(1, weight=1)

        ctrl = tk.Frame(izq, bg=PANEL, padx=10, pady=8)
        ctrl.grid(row=0, column=0, sticky="ew")
        tk.Label(ctrl, text="Ventana de tiempo", bg=PANEL, fg=APAGADO,
                 font=F_NORMAL).pack(side="left")
        combo_v = ttk.Combobox(ctrl, textvariable=self.ventana_var, state="readonly",
                               values=list(VENTANAS), width=14)
        combo_v.pack(side="left", padx=8)
        combo_v.bind("<<ComboboxSelected>>", lambda e: self._redibujar(forzar=True))
        tk.Checkbutton(ctrl, text="Pausar gráfica", variable=self.pausa_var,
                       command=lambda: self._redibujar(forzar=True),
                       bg=PANEL, fg=TEXTO, selectcolor=PANEL2, activebackground=PANEL,
                       activeforeground=TEXTO, font=F_NORMAL).pack(side="left", padx=12)

        self.fig = Figure(figsize=(8, 5), dpi=100, facecolor=PANEL)
        self.ax_t, self.ax_d = self.fig.subplots(
            2, 1, sharex=True, gridspec_kw={"height_ratios": [3, 2], "hspace": 0.08})
        self.fig.subplots_adjust(left=0.08, right=0.98, top=0.96, bottom=0.10)
        for ax in (self.ax_t, self.ax_d):
            ax.set_facecolor(PANEL)
            ax.grid(True, color=BORDE, linestyle="--", alpha=0.7)
            ax.tick_params(colors=APAGADO)
            for lado in ax.spines.values():
                lado.set_color(BORDE)
        self.ax_t.set_ylabel("Temperatura (°C)", color=APAGADO)
        self.ax_d.set_ylabel("Duty cycle (%)", color=APAGADO)
        self.ax_d.set_xlabel("Tiempo (s)", color=APAGADO)
        self.ax_d.set_ylim(-5, 135)                  # Margen superior para la leyenda
        self.ax_d.set_yticks([0, 25, 50, 75, 100])
        self.ax_t.tick_params(labelbottom=False)

        self.l_T, = self.ax_t.plot([], [], color=ROJO, linewidth=2, label="Temperatura")
        self.l_sp, = self.ax_t.plot([], [], color=NARANJA, linestyle="--", linewidth=1.5,
                                    label="Setpoint")
        self.l_ssr, = self.ax_d.plot([], [], color=MORADO, linewidth=1.8, label="Lámparas (SSR)")
        self.l_fan, = self.ax_d.plot([], [], color=CIAN, linewidth=1.8, label="Cooler")
        for ax in (self.ax_t, self.ax_d):
            ax.legend(loc="upper left", ncol=2, facecolor=PANEL2, edgecolor=BORDE,
                      labelcolor=TEXTO)

        self.canvas = FigureCanvasTkAgg(self.fig, master=izq)
        self.canvas.get_tk_widget().configure(bg=PANEL, highlightthickness=0)
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew", padx=6, pady=(0, 6))

        # --- Panel lateral ---
        der = tk.Frame(principal, bg=PANEL, width=330, padx=16, pady=14,
                       highlightthickness=1, highlightbackground=BORDE)
        der.grid(row=0, column=1, sticky="ns", padx=(0, 6))
        der.grid_propagate(False)

        tk.Label(der, text="SESIÓN", bg=PANEL, fg=APAGADO, font=F_TITULO).pack(anchor="w")
        self.lbl_sesion = tk.Label(der, text="Sin datos", bg=PANEL, fg=TEXTO,
                                   font=F_MONO, justify="left")
        self.lbl_sesion.pack(anchor="w", pady=(4, 14))

        tk.Label(der, text="MÉTRICAS DEL CONTROL AUTOMÁTICO", bg=PANEL, fg=APAGADO,
                 font=F_TITULO).pack(anchor="w")
        self.txt_metricas = tk.Text(der, height=11, width=36, bg=PANEL2, fg=TEXTO,
                                    font=("Consolas", 9), relief="flat", wrap="word",
                                    padx=8, pady=8, highlightthickness=0)
        self.txt_metricas.pack(fill="x", pady=(4, 10))
        self._escribir_metricas("Pulsa «Calcular métricas» o desconecta\n"
                                "para analizar el último tramo automático.")

        boton(der, "Calcular métricas", self._mostrar_metricas, AZUL).pack(fill="x", pady=3)
        boton(der, "Exportar (Excel + PDF + PNG)", self._exportar, VERDE).pack(fill="x", pady=3)
        boton(der, "Abrir carpeta de registros", self._abrir_registros, PANEL2).pack(fill="x", pady=3)
        boton(der, "Limpiar sesión", self._limpiar, PANEL2).pack(fill="x", pady=3)

    # ------------------------------------------------------------------ barra de estado
    def _construir_estado(self):
        self.lbl_pie = tk.Label(self, text="Listo.", bg=PANEL, fg=APAGADO, font=("Segoe UI", 9),
                                anchor="w", padx=14, pady=5)
        self.lbl_pie.grid(row=4, column=0, sticky="ew")

    def _estado(self, texto, color):
        self.lbl_estado.config(text=f"● {texto}", fg=color)

    # ------------------------------------------------------------------ puertos / conexión
    def _actualizar_puertos(self):
        self.puertos, sugerido = detectar_puertos()
        self.combo_puerto["values"] = list(self.puertos)
        if sugerido:
            self.puerto_var.set(sugerido)
        elif self.puertos:
            self.puerto_var.set(next(iter(self.puertos)))
        else:
            self.puerto_var.set("")
        self.lbl_pie.config(text=f"{len(self.puertos)} puerto(s) serie encontrado(s).")

    def _alternar_demo(self):
        """Con la demostración activa, el selector de puerto no se usa."""
        if self.demo_var.get() or serial is None:
            self.combo_puerto.config(state="disabled")
            self.btn_refrescar.config(state="disabled")
        else:
            self.combo_puerto.config(state="readonly")
            self.btn_refrescar.config(state="normal")
        if serial is None:
            self.demo_var.set(True)
            self.chk_demo.config(state="disabled")
            self.lbl_pie.config(text="pyserial no está instalado: solo disponible el modo "
                                     "demostración (pip install pyserial).")

    def _alternar_conexion(self):
        if self.conectado:
            self._desconectar()
        else:
            self._conectar()

    def _conectar(self):
        demo = self.demo_var.get()
        puerto = None
        if not demo:
            etiqueta = self.puerto_var.get()
            puerto = self.puertos.get(etiqueta) or etiqueta.split(" ")[0]
            if not puerto:
                messagebox.showwarning("Sin puerto", "Selecciona un puerto o usa el modo demostración.")
                return

        if self.t and not self.exportado:
            if not messagebox.askyesno(
                    "Sesión sin exportar",
                    "La sesión anterior no se ha exportado (el CSV sí quedó guardado).\n"
                    "¿Descartarla y empezar una nueva?"):
                return

        self._reiniciar_datos()
        self._limpiar_grafica()
        self._limpiar_tarjetas()
        self.banner.grid_remove()
        self._escribir_metricas("Sesión en curso...")

        os.makedirs(CARPETA_REGISTROS, exist_ok=True)
        marca = datetime.now().strftime("%Y%m%d_%H%M%S")
        prefijo = "demo" if demo else "control"
        self.ruta_csv = os.path.join(CARPETA_REGISTROS, f"{prefijo}_{marca}.csv")

        self.cola = queue.Queue()                   # Cola nueva: sin restos de sesiones previas
        self.fuente = FuenteDemo(self.cola) if demo else FuenteSerial(puerto, self.cola)
        self.fuente.start()

        self.conectado = True
        self.t_conexion = time.time()
        self.btn_conectar.config(text="Desconectar", bg=ROJO)
        self.combo_puerto.config(state="disabled")
        self.btn_refrescar.config(state="disabled")
        self.chk_demo.config(state="disabled")
        self._estado("Conectado · esperando datos" if not demo else "Demostración", AMBAR if not demo else AZUL)

    def _desconectar(self, mostrar_metricas=True):
        if self.fuente is not None:
            self.fuente.detener()
            self.fuente.join(timeout=1.0)
            self.fuente = None
        self._cerrar_csv()
        self.conectado = False
        self.btn_conectar.config(text="Conectar", bg=VERDE)
        self.chk_demo.config(state="normal" if serial is not None else "disabled")
        self._alternar_demo()
        self._estado("Desconectado", APAGADO)
        if mostrar_metricas and self.t:
            self._mostrar_metricas()

    # ------------------------------------------------------------------ CSV continuo
    def _escribir_csv(self, d):
        if self.escritor_csv is None:
            self.archivo_csv = open(self.ruta_csv, "w", newline="", encoding="utf-8")
            self.escritor_csv = csv.writer(self.archivo_csv)
            self.escritor_csv.writerow(COLUMNAS)
        self.escritor_csv.writerow(d)

    def _cerrar_csv(self):
        if self.archivo_csv is not None:
            self.archivo_csv.close()
        self.archivo_csv = None
        self.escritor_csv = None

    # ------------------------------------------------------------------ bucle de datos
    def _sondear(self):
        self.after(100, self._sondear)              # Se reprograma primero: nunca se detiene

        if not self.conectado:
            # Restos de un hilo ya detenido: se descartan para no reabrir el CSV cerrado
            while not self.cola.empty():
                try:
                    self.cola.get_nowait()
                except queue.Empty:
                    break
            return

        nuevos, ultimo, error = 0, None, None
        try:
            while True:
                tipo, carga = self.cola.get_nowait()
                if tipo == "dato":
                    self._agregar_muestra(carga)
                    ultimo = carga
                    nuevos += 1
                elif tipo == "error":
                    error = carga
        except queue.Empty:
            pass

        if self.archivo_csv is not None and nuevos:
            self.archivo_csv.flush()                # No perder datos si el programa se cierra mal

        if ultimo is not None:
            self._actualizar_tarjetas(ultimo)
            self._actualizar_sesion()
            if self.conectado:
                self._estado("Recibiendo datos" if not self.demo_var.get() else "Demostración",
                             VERDE if not self.demo_var.get() else AZUL)

        if nuevos and time.time() - self.ultimo_dibujo >= PERIODO_REFRESCO_S:
            self._redibujar()

        self._vigilar_silencio()

        if error:
            self._desconectar(mostrar_metricas=bool(self.t))
            messagebox.showerror("Error de conexión", error)

    def _vigilar_silencio(self):
        """Avisa si la conexión está abierta pero no llegan datos."""
        if not self.conectado or self.demo_var.get():
            return
        referencia = self.t_ultimo_dato or self.t_conexion
        if referencia and time.time() - referencia > SIN_DATOS_AVISO_S:
            if self.t_ultimo_dato:
                self._estado("Sin datos recientes", AMBAR)
            else:
                self._estado("Conectado, pero no llegan datos (¿firmware correcto?)", AMBAR)

    def _agregar_muestra(self, d):
        t, temp, sp, cooler, ssr, modo, alarma = d
        self.t.append(t); self.T.append(temp); self.sp.append(sp)
        self.fan.append(cooler); self.ssr.append(ssr)
        self.modo.append(modo); self.alarma.append(alarma)
        self.t_ultimo_dato = time.time()
        self.exportado = False
        if not alarma:
            self.t_min = temp if self.t_min is None else min(self.t_min, temp)
            self.t_max = temp if self.t_max is None else max(self.t_max, temp)
        self._escribir_csv(d)

    # ------------------------------------------------------------------ actualización visual
    def _actualizar_tarjetas(self, d):
        _, temp, sp, cooler, ssr, modo, alarma = d

        if alarma:
            color_t = ROJO
        elif abs(temp - sp) <= 2.0:
            color_t = VERDE
        elif temp < sp:
            color_t = AZUL
        else:
            color_t = ROJO
        self.c_temp.poner(f"{temp:.1f} °C", f"{temp - sp:+.1f} °C respecto al setpoint", color_t)
        self.c_sp.poner(f"{sp:.1f} °C",
                        f"Rango proporcional: {sp - RANGO_PROPORCIONAL_C:.0f}–{sp + RANGO_PROPORCIONAL_C:.0f} °C")
        if modo:
            self.c_modo.poner("AUTO", "Control proporcional", VERDE)
        else:
            self.c_modo.poner("MANUAL", "Duty fijado con potenciómetros", AMBAR)
        self.c_ssr.poner(f"{ssr} %", "Calienta" if ssr > 0 else "Apagadas", MORADO, ssr)
        self.c_fan.poner(f"{cooler} %", "Enfría" if cooler > 0 else "Apagado", CIAN, cooler)

        if alarma:
            causa = "SOBRETEMPERATURA" if temp >= TEMP_CORTE_C - 5 else "FALLA DEL SENSOR / SIN LECTURA"
            self.banner.config(text=f"⚠  ALARMA · {causa} — lámparas apagadas y cooler al 100 %")
            self.banner.grid()
        else:
            self.banner.grid_remove()

    def _actualizar_sesion(self):
        dur = self.t[-1] if self.t else 0
        mm, ss = divmod(int(dur), 60)
        t_min = "--" if self.t_min is None else f"{self.t_min:.1f} °C"
        t_max = "--" if self.t_max is None else f"{self.t_max:.1f} °C"
        auto = sum(self.modo)
        pct_auto = 100 * auto / len(self.modo) if self.modo else 0
        self.lbl_sesion.config(
            text=(f"Duración   {mm:02d}:{ss:02d}\n"
                  f"Muestras   {len(self.t)}\n"
                  f"T mínima   {t_min}\n"
                  f"T máxima   {t_max}\n"
                  f"En AUTO    {pct_auto:.0f} %\n"
                  f"CSV        {os.path.basename(self.ruta_csv or '--')}"))

    def _limpiar_grafica(self):
        for l in (self.l_T, self.l_sp, self.l_ssr, self.l_fan):
            l.set_data([], [])
        self.ax_t.set_xlim(0, 10)
        self.ax_t.set_ylim(0, 100)
        self.canvas.draw_idle()
        self.lbl_sesion.config(text="Sin datos")

    def _redibujar(self, forzar=False):
        if len(self.t) < 2:
            return
        if self.pausa_var.get() and not forzar:
            return
        self.ultimo_dibujo = time.time()

        ventana = VENTANAS[self.ventana_var.get()]
        i0 = bisect_left(self.t, self.t[-1] - ventana) if ventana else 0
        xs = self.t[i0:]
        Ts, sps = self.T[i0:], self.sp[i0:]

        self.l_T.set_data(xs, Ts)
        self.l_sp.set_data(xs, sps)
        self.l_ssr.set_data(xs, self.ssr[i0:])
        self.l_fan.set_data(xs, self.fan[i0:])

        izq = xs[0]
        self.ax_t.set_xlim(izq, max(xs[-1], izq + 10))
        self.ax_t.set_ylim(*limites_temperatura(Ts, sps))
        self.canvas.draw_idle()

    # ------------------------------------------------------------------ métricas / exportación
    def _dataframe(self):
        return pd.DataFrame({
            COLUMNAS[0]: self.t, COLUMNAS[1]: self.T, COLUMNAS[2]: self.sp,
            COLUMNAS[3]: self.fan, COLUMNAS[4]: self.ssr,
            COLUMNAS[5]: self.modo, COLUMNAS[6]: self.alarma})

    def _escribir_metricas(self, texto):
        self.txt_metricas.config(state="normal")
        self.txt_metricas.delete("1.0", "end")
        self.txt_metricas.insert("1.0", texto)
        self.txt_metricas.config(state="disabled")

    def _mostrar_metricas(self):
        if not self.t:
            self._escribir_metricas("Aún no hay datos.")
            return
        self._escribir_metricas(formatear_metricas(calcular_metricas(self._dataframe())))

    def _exportar(self):
        if not self.t:
            messagebox.showinfo("Exportar", "Aún no hay datos para exportar.")
            return
        os.makedirs(CARPETA_REGISTROS, exist_ok=True)
        base = os.path.splitext(self.ruta_csv)[0] if self.ruta_csv else \
            os.path.join(CARPETA_REGISTROS, datetime.now().strftime("control_%Y%m%d_%H%M%S"))
        try:
            df = self._dataframe()
            archivos = exportar_sesion(df, base, calcular_metricas(df))
        except ImportError:
            messagebox.showerror("Falta openpyxl", "Ejecuta: pip install -r requirements.txt")
            return
        except Exception as e:
            messagebox.showerror("Error al exportar", str(e))
            return
        self.exportado = True
        nombres = "\n".join(f"• {os.path.basename(a)}" for a in archivos)
        messagebox.showinfo("Exportación completada",
                            f"Archivos guardados en:\n{CARPETA_REGISTROS}\n\n{nombres}")

    def _abrir_registros(self):
        os.makedirs(CARPETA_REGISTROS, exist_ok=True)
        abrir_carpeta(CARPETA_REGISTROS)

    def _limpiar(self):
        if self.conectado:
            messagebox.showinfo("Limpiar", "Desconecta antes de limpiar la sesión.")
            return
        if self.t and not self.exportado:
            if not messagebox.askyesno("Limpiar sesión",
                                       "La sesión no se ha exportado (el CSV sí está guardado).\n"
                                       "¿Limpiar de todos modos?"):
                return
        self._reiniciar_datos()
        self._limpiar_grafica()
        self._limpiar_tarjetas()
        self.banner.grid_remove()
        self._escribir_metricas("Sesión limpia.")

    # ------------------------------------------------------------------ cierre
    def _cerrar(self):
        if self.conectado:
            self._desconectar(mostrar_metricas=False)
        self._cerrar_csv()
        self.destroy()


# ============================================================================
#  PUNTO DE ENTRADA
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="Monitor gráfico del control térmico ESP32")
    parser.add_argument("--demo", action="store_true", help="Inicia en modo demostración")
    args = parser.parse_args()

    if sys.platform.startswith("win"):               # Texto nítido en pantallas de alta densidad
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass

    App(demo=args.demo).mainloop()


if __name__ == "__main__":
    main()
