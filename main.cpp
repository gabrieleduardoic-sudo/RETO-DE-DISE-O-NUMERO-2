/**
 * ============================================================================
 *  Laboratorio 2 - Microcontroladores
 *  Sistema de monitorización y control de temperatura con ESP32
 * ============================================================================
 *  Variable controlada : temperatura (termocupla K + MAX6675)
 *  Actuador que SUBE   : 2 lámparas de 50 W controladas con un SSR
 *                        (PWM por software, ventana lenta de 5 s)
 *  Actuador que BAJA   : cooler de 4 hilos (PWM por hardware, 25 kHz)
 *  Interfaz            : OLED SSD1306 (I2C), 3 pulsadores, 2 potenciómetros
 *
 *  Modos de operación (pulsador MODO alterna entre ellos):
 *    - MANUAL    : los potenciómetros fijan el duty del SSR y del cooler.
 *    - AUTOMATICO: el duty se calcula con dos rectas alrededor del setpoint.
 *
 *  Rectas del modo automático (SP = setpoint):
 *    Lámpara : 100 % en (SP - RANGO)  ->  DUTY_MIN_LAMPARA % en (SP - BANDA)
 *              0 % para T >= SP - BANDA
 *    Cooler  : 0 % para T <= SP
 *              DUTY_MIN_VENTILADOR % en SP  ->  100 % en (SP + RANGO)
 *
 *  Protecciones (activas en AMBOS modos):
 *    - Fallo del sensor (termocupla desconectada): SSR apagado, cooler 100 %.
 *    - Sobretemperatura (> TEMP_CORTE_C): SSR apagado, cooler 100 %,
 *      con histéresis de rearme para evitar oscilar en el límite.
 *
 *  Pantalla OLED 128x64 (distribución):
 *    +----------------------------------+
 *    | AUTO                    SP 60.0C |  <- cabecera (en alarma parpadea)
 *    |                                  |
 *    | 58.3 °C                  ^  -1.7 |  <- temperatura, tendencia y error
 *    |                                  |
 *    | LAMP [=======......]        62%  |  <- duty de las lámparas (SSR)
 *    | FAN  [..............]         0%  |  <- duty del cooler
 *    +----------------------------------+
 *    La flecha indica si la temperatura sube, baja o está estable.
 *    El valor junto a ella es el error T - SP (negativo = por debajo).
 *
 *  Salida serial (CSV a 115200 baudios, 4 Hz):
 *    Tiempo_ms,Temperatura_C,Setpoint_C,Duty_Cooler,Duty_SSR,Modo,Alarma
 *    Modo  : 0 = manual, 1 = automático
 *    Alarma: 0 = normal, 1 = fallo de sensor o sobretemperatura
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "max6675.h"

// ============================================================================
//  DEFINICIÓN DE PINES
// ============================================================================
// MAX6675 (SPI por software: la librería maneja los tres pines)
#define PIN_TERMOCUPLA_CS   5
#define PIN_TERMOCUPLA_SCK  18
#define PIN_TERMOCUPLA_SO   19

// Actuadores
#define PIN_SSR             26   // Salida digital hacia el SSR (lámparas)
#define PIN_VENTILADOR      27   // Salida PWM hacia el cooler

// Potenciómetros (ADC1: funciona aunque se use WiFi, ADC2 no)
#define PIN_POT_1           32   // Manual: duty del SSR (lámparas)
#define PIN_POT_2           33   // Manual: duty del ventilador

// Pulsadores (con pull-up interno: LOW = presionado)
#define PIN_BTN_MODO        25   // Alterna MANUAL / AUTOMATICO
#define PIN_BTN_UP          15   // Sube el setpoint
#define PIN_BTN_DOWN        4    // Baja el setpoint

// ============================================================================
//  PARÁMETROS DE CONTROL
// ============================================================================
float setpoint = 60.0;                    // Setpoint inicial [°C]
const float SETPOINT_MIN = 50.0;          // Límite inferior ajustable [°C]
// Se limita a 100 °C para que siempre quede por debajo del corte de seguridad.
const float SETPOINT_MAX = 100.0;         // Límite superior ajustable [°C]

// Desplazamiento (banda muerta) entre el setpoint y el apagado de la lámpara.
// Evita que la lámpara siga encendida justo en el setpoint.
const float BANDA_MUERTA        = 1.0;    // [°C]
// Ancho de cada recta proporcional [°C]
const float RANGO_PROPORCIONAL  = 20.0;

// Duty mínimo con el que cada actuador realmente arranca/actúa.
// Medir en la caracterización y reemplazar (ver Figura 1 del enunciado,
// donde ese mínimo es 40 %). Con 0 las rectas parten desde cero.
const int DUTY_MIN_LAMPARA      = 0;      // [%] duty de la lámpara en SP - BANDA
const int DUTY_MIN_VENTILADOR   = 0;      // [%] duty del cooler en SP

// Protecciones
const float   TEMP_CORTE_C         = 110.0;  // Corte duro por sobretemperatura
const float   TEMP_REARME_HIST_C   = 5.0;    // Se rearma al bajar de CORTE - 5 °C
const uint8_t MAX_FALLOS_SENSOR    = 4;      // Lecturas NaN consecutivas = fallo

// Filtro exponencial de la temperatura: y = (1-a)*y + a*lectura
const float ALFA_FILTRO = 0.3;

// ============================================================================
//  PARÁMETROS DE HARDWARE
// ============================================================================
// PWM del cooler
const uint32_t PWM_FRECUENCIA_HZ  = 25000;  // 25 kHz: estándar de ventiladores 4 hilos
const uint8_t  PWM_RESOLUCION_BIT = 8;      // 0..255
const uint8_t  PWM_CANAL          = 0;      // Canal LEDC (solo core 2.x)
const int      PWM_MAX            = (1 << PWM_RESOLUCION_BIT) - 1;

// El SSR de AC conmuta en el cruce por cero: no admite PWM rápido.
// Se usa una ventana lenta: dentro de cada ventana la salida está
// encendida durante (duty % * VENTANA) y apagada el resto.
const unsigned long VENTANA_SSR_MS = 5000;

// Potenciómetros
const int ADC_MAX            = 4095;   // 12 bits
const int ADC_MUESTRAS       = 8;      // Promedio para reducir ruido
const int POT_ZONA_MUERTA_PCT = 3;     // <3 % -> 0 %, >97 % -> 100 %

// OLED
const uint8_t OLED_ANCHO     = 128;
const uint8_t OLED_ALTO      = 64;
const uint8_t OLED_DIRECCION = 0x3C;

// Diseño de la pantalla (píxeles)
const uint8_t OLED_CABECERA_ALTO = 12;   // Barra superior con modo y setpoint
const uint8_t OLED_TEMP_Y        = 17;   // Fila de la temperatura grande
const uint8_t OLED_BARRA_X       = 27;   // Inicio de las barras de duty
const uint8_t OLED_BARRA_ANCHO   = 75;
const uint8_t OLED_BARRA_ALTO    = 9;
const uint8_t OLED_FILA_LAMP_Y   = 42;
const uint8_t OLED_FILA_FAN_Y    = 54;
const unsigned long OLED_PARPADEO_MS = 500;   // Parpadeo de la cabecera en alarma

// Flecha de tendencia: se compara la temperatura contra la de hace 4 s
const unsigned long TENDENCIA_VENTANA_MS = 4000;
const float         TENDENCIA_UMBRAL_C   = 0.25;   // Cambio mínimo para considerar sube/baja

// ============================================================================
//  TEMPORIZADORES (todo no bloqueante)
// ============================================================================
const unsigned long INTERVALO_SENSOR_MS   = 250;  // MAX6675 convierte en ~220 ms
const unsigned long INTERVALO_CONTROL_MS  = 100;
const unsigned long INTERVALO_OLED_MS     = 200;
const unsigned long INTERVALO_DEBOUNCE_MS = 50;
const unsigned long INTERVALO_SERIAL_MS   = 250;

// Repetición automática al mantener presionado UP/DOWN
const unsigned long RETARDO_REPETICION_MS   = 500;
const unsigned long INTERVALO_REPETICION_MS = 150;

unsigned long tInicioVentanaSSR = 0;
unsigned long tUltimaLecturaSensor = 0;
unsigned long tUltimoControl = 0;
unsigned long tUltimaOLED = 0;
unsigned long tUltimoBoton = 0;
unsigned long tUltimoSerial = 0;

unsigned long tPresionadoUp = 0;
unsigned long tUltimaRepeticionUp = 0;
unsigned long tPresionadoDown = 0;
unsigned long tUltimaRepeticionDown = 0;

// ============================================================================
//  OBJETOS Y VARIABLES DE ESTADO
// ============================================================================
MAX6675 termocupla(PIN_TERMOCUPLA_SCK, PIN_TERMOCUPLA_CS, PIN_TERMOCUPLA_SO);
Adafruit_SSD1306 display(OLED_ANCHO, OLED_ALTO, &Wire, -1);

enum Modo { MODO_MANUAL = 0, MODO_AUTOMATICO = 1 };
Modo modo = MODO_MANUAL;              // Arranca en manual por seguridad

float temperaturaActual = 0.0;        // Temperatura filtrada [°C]
bool  hayPrimeraLectura = false;      // Para inicializar el filtro
int   dutySSR = 0;                    // [%]
int   dutyVentilador = 0;             // [%]

uint8_t fallosSensor = 0;             // NaN consecutivos
bool    falloSensor = false;          // Fallo confirmado del sensor
bool    sobretemperatura = false;     // Corte por sobretemperatura (con latch)
bool    alarma = false;               // Cualquiera de las dos anteriores
bool    oledDisponible = false;

// Estado anterior de los pulsadores (detección de flancos)
bool estadoAnteriorBtnModo = HIGH;
bool estadoAnteriorBtnUp   = HIGH;
bool estadoAnteriorBtnDown = HIGH;

// ============================================================================
//  PROTOTIPOS
// ============================================================================
void leerBotones();
void leerSensor();
void actualizarControl();
void calcularControlAuto();
void calcularControlManual();
void controlarSSR_NoBlock(int duty);
void controlarVentilador(int duty);
void enviarSerial(unsigned long ahora);
void actualizarDisplay();
void mostrarPantallaInicio();
void dibujarCabecera();
void dibujarTemperatura();
void dibujarFilaActuador(const char* etiqueta, uint8_t y, int porcentaje);
void dibujarBarra(int x, int y, int ancho, int alto, int porcentaje);
void dibujarTextoDerecha(const char* texto, int xDerecha, int y);
int8_t tendenciaTemperatura();
int  leerPotenciometroPct(uint8_t pin);
int  interpolar(float x, float x0, float x1, int y0, int y1);
void escribirPWM(int valor);

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Tiempo_ms,Temperatura_C,Setpoint_C,Duty_Cooler,Duty_SSR,Modo,Alarma");

    pinMode(PIN_BTN_MODO, INPUT_PULLUP);
    pinMode(PIN_BTN_UP,   INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);

    pinMode(PIN_POT_1, INPUT);
    pinMode(PIN_POT_2, INPUT);

    // El SSR debe quedar apagado desde el primer instante
    pinMode(PIN_SSR, OUTPUT);
    digitalWrite(PIN_SSR, LOW);

    // PWM del cooler. La API cambió entre las versiones 2.x y 3.x del core.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(PIN_VENTILADOR, PWM_FRECUENCIA_HZ, PWM_RESOLUCION_BIT);
#else
    ledcSetup(PWM_CANAL, PWM_FRECUENCIA_HZ, PWM_RESOLUCION_BIT);
    ledcAttachPin(PIN_VENTILADOR, PWM_CANAL);
#endif
    escribirPWM(0);

    // OLED. Si falla, el sistema sigue funcionando (el control no depende de ella).
    Wire.begin();
    Wire.setClock(400000);  // I2C rápido: el refresco baja de ~90 ms a ~25 ms
    oledDisponible = display.begin(SSD1306_SWITCHCAPVCC, OLED_DIRECCION);
    if (oledDisponible) {
        mostrarPantallaInicio();
    } else {
        Serial.println(F("Fallo al inicializar OLED (se continua sin pantalla)"));
    }

    // El MAX6675 necesita ~500 ms tras el encendido antes de la primera lectura.
    // Se aprovecha el tiempo para que se vea la pantalla de inicio.
    delay(1200);
}

// ============================================================================
//  LOOP PRINCIPAL
// ============================================================================
void loop() {
    unsigned long ahora = millis();

    // 1) Pulsadores (con antirrebote por muestreo cada 50 ms)
    if (ahora - tUltimoBoton >= INTERVALO_DEBOUNCE_MS) {
        tUltimoBoton = ahora;
        leerBotones();
    }

    // 2) Lectura del sensor
    if (ahora - tUltimaLecturaSensor >= INTERVALO_SENSOR_MS) {
        tUltimaLecturaSensor = ahora;
        leerSensor();
    }

    // 3) Cálculo del control (automático o manual) y protecciones
    if (ahora - tUltimoControl >= INTERVALO_CONTROL_MS) {
        tUltimoControl = ahora;
        actualizarControl();
    }

    // 4) Salidas (se llaman en cada iteración: el SSR necesita temporización fina)
    controlarSSR_NoBlock(dutySSR);
    controlarVentilador(dutyVentilador);

    // 5) Telemetría y pantalla
    if (ahora - tUltimoSerial >= INTERVALO_SERIAL_MS) {
        tUltimoSerial = ahora;
        enviarSerial(ahora);
    }
    if (ahora - tUltimaOLED >= INTERVALO_OLED_MS) {
        tUltimaOLED = ahora;
        actualizarDisplay();
    }
}

// ============================================================================
//  SENSOR
// ============================================================================
/**
 * Lee la termocupla, detecta fallos y filtra la señal.
 * - Si llegan MAX_FALLOS_SENSOR lecturas NaN seguidas se declara fallo
 *   (termocupla desconectada o ruido). No se conserva la última temperatura
 *   como válida: el control pasa a estado seguro.
 * - Se aplica un filtro exponencial para reducir el ruido de cuantización
 *   (0,25 °C) y el ruido eléctrico de las lámparas.
 */
void leerSensor() {
    float lectura = termocupla.readCelsius();

    if (isnan(lectura)) {
        if (fallosSensor < 255) fallosSensor++;
        if (fallosSensor >= MAX_FALLOS_SENSOR) falloSensor = true;
        return;
    }

    fallosSensor = 0;
    falloSensor = false;

    if (!hayPrimeraLectura) {
        temperaturaActual = lectura;   // Inicializa el filtro sin transitorio
        hayPrimeraLectura = true;
    } else {
        temperaturaActual = (1.0f - ALFA_FILTRO) * temperaturaActual + ALFA_FILTRO * lectura;
    }

    // Sobretemperatura con latch: se activa en CORTE y se libera 5 °C por debajo
    if (temperaturaActual >= TEMP_CORTE_C) {
        sobretemperatura = true;
    } else if (temperaturaActual <= TEMP_CORTE_C - TEMP_REARME_HIST_C) {
        sobretemperatura = false;
    }
}

// ============================================================================
//  CONTROL
// ============================================================================
/**
 * Calcula los duty según el modo y luego aplica las protecciones,
 * que tienen prioridad sobre cualquier modo.
 */
void actualizarControl() {
    if (modo == MODO_AUTOMATICO) {
        calcularControlAuto();
    } else {
        calcularControlManual();
    }

    // Protecciones: estado seguro = sin calor y máxima refrigeración
    alarma = falloSensor || sobretemperatura || !hayPrimeraLectura;
    if (alarma) {
        dutySSR = 0;
        dutyVentilador = 100;
    }
}

/**
 * Modo automático: dos rectas (control proporcional) alrededor del setpoint.
 */
void calcularControlAuto() {
    const float T = temperaturaActual;

    // Lámpara: se apaga en SP - BANDA y llega a 100 % en SP - RANGO
    const float lamparaApagado = setpoint - BANDA_MUERTA;
    const float lamparaMaximo  = setpoint - RANGO_PROPORCIONAL;

    if (T >= lamparaApagado)      dutySSR = 0;
    else if (T <= lamparaMaximo)  dutySSR = 100;
    else dutySSR = interpolar(T, lamparaApagado, lamparaMaximo, DUTY_MIN_LAMPARA, 100);

    // Cooler: arranca en SP y llega a 100 % en SP + RANGO
    const float coolerInicio = setpoint;
    const float coolerMaximo = setpoint + RANGO_PROPORCIONAL;

    if (T <= coolerInicio)        dutyVentilador = 0;
    else if (T >= coolerMaximo)   dutyVentilador = 100;
    else dutyVentilador = interpolar(T, coolerInicio, coolerMaximo, DUTY_MIN_VENTILADOR, 100);
}

/**
 * Modo manual: cada potenciómetro fija el duty de su actuador.
 */
void calcularControlManual() {
    dutySSR        = leerPotenciometroPct(PIN_POT_1);
    dutyVentilador = leerPotenciometroPct(PIN_POT_2);
}

/**
 * Interpolación lineal entre (x0,y0) y (x1,y1), redondeada al entero más cercano.
 * Se redondea (no se trunca) para no sesgar el duty hacia abajo.
 */
int interpolar(float x, float x0, float x1, int y0, int y1) {
    float y = y0 + (x - x0) * (float)(y1 - y0) / (x1 - x0);
    int r = (int)lroundf(y);
    return constrain(r, 0, 100);
}

/**
 * Lee un potenciómetro promediando varias muestras y devuelve 0..100 %.
 * La zona muerta en los extremos garantiza que se alcancen 0 % y 100 %,
 * ya que el ADC del ESP32 es no lineal cerca de los límites.
 */
int leerPotenciometroPct(uint8_t pin) {
    long suma = 0;
    for (int i = 0; i < ADC_MUESTRAS; i++) suma += analogRead(pin);
    float pct = (suma / (float)ADC_MUESTRAS) * 100.0f / ADC_MAX;

    if (pct < POT_ZONA_MUERTA_PCT)         return 0;
    if (pct > 100 - POT_ZONA_MUERTA_PCT)   return 100;
    return (int)lroundf(pct);
}

// ============================================================================
//  PULSADORES (lógica sin cambios respecto a la versión original)
// ============================================================================
void leerBotones() {
    unsigned long ahora = millis();

    // --- Botón MODO: alterna al detectar el flanco de bajada ---
    bool actualModo = digitalRead(PIN_BTN_MODO);
    if (actualModo == LOW && estadoAnteriorBtnModo == HIGH) {
        modo = (modo == MODO_AUTOMATICO) ? MODO_MANUAL : MODO_AUTOMATICO;
    }
    estadoAnteriorBtnModo = actualModo;

    // --- Botón UP: +1 °C al presionar, repite si se mantiene ---
    bool actualUp = digitalRead(PIN_BTN_UP);
    if (actualUp == LOW) {
        if (estadoAnteriorBtnUp == HIGH) {
            setpoint += 1.0;
            if (setpoint > SETPOINT_MAX) setpoint = SETPOINT_MAX;
            tPresionadoUp = ahora;
            tUltimaRepeticionUp = ahora;
        } else if (ahora - tPresionadoUp >= RETARDO_REPETICION_MS &&
                   ahora - tUltimaRepeticionUp >= INTERVALO_REPETICION_MS) {
            setpoint += 1.0;
            if (setpoint > SETPOINT_MAX) setpoint = SETPOINT_MAX;
            tUltimaRepeticionUp = ahora;
        }
    }
    estadoAnteriorBtnUp = actualUp;

    // --- Botón DOWN: -1 °C al presionar, repite si se mantiene ---
    bool actualDown = digitalRead(PIN_BTN_DOWN);
    if (actualDown == LOW) {
        if (estadoAnteriorBtnDown == HIGH) {
            setpoint -= 1.0;
            if (setpoint < SETPOINT_MIN) setpoint = SETPOINT_MIN;
            tPresionadoDown = ahora;
            tUltimaRepeticionDown = ahora;
        } else if (ahora - tPresionadoDown >= RETARDO_REPETICION_MS &&
                   ahora - tUltimaRepeticionDown >= INTERVALO_REPETICION_MS) {
            setpoint -= 1.0;
            if (setpoint < SETPOINT_MIN) setpoint = SETPOINT_MIN;
            tUltimaRepeticionDown = ahora;
        }
    }
    estadoAnteriorBtnDown = actualDown;
}

// ============================================================================
//  ACTUADORES
// ============================================================================
/**
 * PWM lento por software para el SSR (no bloqueante).
 * Periodo fijo de VENTANA_SSR_MS. Se avanza el inicio con += para que
 * el periodo sea exacto y no acumule deriva.
 */
void controlarSSR_NoBlock(int duty) {
    unsigned long ahora = millis();

    while (ahora - tInicioVentanaSSR >= VENTANA_SSR_MS) {
        tInicioVentanaSSR += VENTANA_SSR_MS;
    }

    unsigned long tiempoEncendido = (VENTANA_SSR_MS * (unsigned long)duty) / 100;
    digitalWrite(PIN_SSR, (ahora - tInicioVentanaSSR) < tiempoEncendido ? HIGH : LOW);
}

/** Convierte duty 0..100 % al valor de PWM y lo escribe. */
void controlarVentilador(int duty) {
    duty = constrain(duty, 0, 100);
    escribirPWM((duty * PWM_MAX + 50) / 100);   // +50: redondeo entero
}

/** Escritura PWM compatible con core 2.x (por canal) y 3.x (por pin). */
void escribirPWM(int valor) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(PIN_VENTILADOR, valor);
#else
    ledcWrite(PWM_CANAL, valor);
#endif
}

// ============================================================================
//  TELEMETRÍA Y PANTALLA
// ============================================================================
void enviarSerial(unsigned long ahora) {
    Serial.print(ahora);                    Serial.print(',');
    Serial.print(temperaturaActual, 1);     Serial.print(',');
    Serial.print(setpoint, 1);              Serial.print(',');
    Serial.print(dutyVentilador);           Serial.print(',');
    Serial.print(dutySSR);                  Serial.print(',');
    Serial.print((int)modo);                Serial.print(',');
    Serial.println(alarma ? 1 : 0);
}

/** Pantalla de bienvenida mientras el MAX6675 se estabiliza. */
void mostrarPantallaInicio() {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(22, 6);   display.print("CONTROL");
    display.setCursor(22, 24);  display.print("TERMICO");
    display.drawFastHLine(0, 44, OLED_ANCHO, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(25, 51);  display.print("Lab 2 - ESP32");
    display.display();
}

/** Dibuja un cuadro completo. Se llama cada INTERVALO_OLED_MS. */
void actualizarDisplay() {
    if (!oledDisponible) return;

    display.clearDisplay();
    dibujarCabecera();
    dibujarTemperatura();
    dibujarFilaActuador("LAMP", OLED_FILA_LAMP_Y, dutySSR);
    dibujarFilaActuador("FAN",  OLED_FILA_FAN_Y,  dutyVentilador);
    display.display();
}

/**
 * Barra superior: modo a la izquierda y setpoint a la derecha.
 * En alarma muestra la causa y parpadea alternando colores, para que se
 * note incluso de lejos.
 */
void dibujarCabecera() {
    const bool fondoBlanco = !alarma || ((millis() / OLED_PARPADEO_MS) % 2 == 0);

    display.fillRect(0, 0, OLED_ANCHO, OLED_CABECERA_ALTO,
                     fondoBlanco ? SSD1306_WHITE : SSD1306_BLACK);
    if (!fondoBlanco) {
        display.drawRect(0, 0, OLED_ANCHO, OLED_CABECERA_ALTO, SSD1306_WHITE);
    }
    display.setTextColor(fondoBlanco ? SSD1306_BLACK : SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(4, 2);

    if (alarma) {
        display.print((falloSensor || !hayPrimeraLectura) ? "! ERR SENSOR" : "! SOBRETEMP");
    } else {
        display.print(modo == MODO_AUTOMATICO ? "AUTO" : "MANUAL");
        char buf[14];
        snprintf(buf, sizeof(buf), "SP %.1fC", setpoint);
        dibujarTextoDerecha(buf, OLED_ANCHO - 4, 2);
    }
}

/**
 * Temperatura en tamaño grande con su unidad, flecha de tendencia y error
 * respecto al setpoint. Si no hay lectura válida muestra guiones.
 */
void dibujarTemperatura() {
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(3);
    display.setCursor(2, OLED_TEMP_Y);

    if (!hayPrimeraLectura || falloSensor) {
        display.print("--.-");
        return;
    }

    // Con 3 cifras enteras (>= 100 °C) se omite el decimal para que quepa
    char buf[8];
    if (temperaturaActual >= 100.0f) snprintf(buf, sizeof(buf), "%d", (int)lroundf(temperaturaActual));
    else                             snprintf(buf, sizeof(buf), "%.1f", temperaturaActual);
    display.print(buf);

    // Unidad: círculo de grado + "C" (la fuente por defecto no trae el símbolo °)
    const int xFin = 2 + (int)strlen(buf) * 18;      // Cada carácter mide 18 px a tamaño 3
    display.drawCircle(xFin + 3, OLED_TEMP_Y + 3, 2, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(xFin + 8, OLED_TEMP_Y);
    display.print("C");

    // Flecha de tendencia (centrada en x = 112)
    const int8_t tendencia = tendenciaTemperatura();
    if (tendencia > 0) {
        display.fillTriangle(112, 17, 103, 29, 121, 29, SSD1306_WHITE);   // Sube
    } else if (tendencia < 0) {
        display.fillTriangle(103, 17, 121, 17, 112, 29, SSD1306_WHITE);   // Baja
    } else {
        display.fillRect(103, 21, 19, 4, SSD1306_WHITE);                   // Estable
    }

    // Error respecto al setpoint, bajo la flecha
    char err[10];
    snprintf(err, sizeof(err), "%+.1f", temperaturaActual - setpoint);
    display.setTextSize(1);
    dibujarTextoDerecha(err, OLED_ANCHO - 1, 32);
}

/**
 * Devuelve +1 (sube), -1 (baja) o 0 (estable).
 * Se actualiza cada TENDENCIA_VENTANA_MS comparando con la muestra anterior,
 * de modo que el ruido del sensor no haga parpadear la flecha.
 */
int8_t tendenciaTemperatura() {
    static bool inicializado = false;
    static unsigned long tReferencia = 0;
    static float tempReferencia = 0.0f;
    static int8_t tendencia = 0;

    const unsigned long ahora = millis();
    if (!inicializado) {
        inicializado = true;
        tReferencia = ahora;
        tempReferencia = temperaturaActual;
    } else if (ahora - tReferencia >= TENDENCIA_VENTANA_MS) {
        const float delta = temperaturaActual - tempReferencia;
        if (delta > TENDENCIA_UMBRAL_C)       tendencia = 1;
        else if (delta < -TENDENCIA_UMBRAL_C) tendencia = -1;
        else                                  tendencia = 0;
        tReferencia = ahora;
        tempReferencia = temperaturaActual;
    }
    return tendencia;
}

/** Fila de un actuador: etiqueta, barra proporcional y porcentaje. */
void dibujarFilaActuador(const char* etiqueta, uint8_t y, int porcentaje) {
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, y + 1);
    display.print(etiqueta);

    dibujarBarra(OLED_BARRA_X, y, OLED_BARRA_ANCHO, OLED_BARRA_ALTO, porcentaje);

    char buf[6];
    snprintf(buf, sizeof(buf), "%d%%", porcentaje);
    dibujarTextoDerecha(buf, OLED_ANCHO - 1, y + 1);
}

/** Barra horizontal con contorno y relleno proporcional (0..100 %). */
void dibujarBarra(int x, int y, int ancho, int alto, int porcentaje) {
    porcentaje = constrain(porcentaje, 0, 100);
    display.drawRect(x, y, ancho, alto, SSD1306_WHITE);
    const int relleno = ((ancho - 4) * porcentaje) / 100;
    if (relleno > 0) {
        display.fillRect(x + 2, y + 2, relleno, alto - 4, SSD1306_WHITE);
    }
}

/** Escribe texto (tamaño 1, 6 px por carácter) alineado a la derecha. */
void dibujarTextoDerecha(const char* texto, int xDerecha, int y) {
    display.setCursor(xDerecha - (int)strlen(texto) * 6 + 1, y);
    display.print(texto);
}
