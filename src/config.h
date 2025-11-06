/*******************************************************************************
 * CONFIG.H - Configuración de Pines y Parámetros
 *
 * Este archivo centraliza toda la configuración del robot seguidor de línea:
 * - Definición de pines GPIO del ESP32-S3
 * - Parámetros de control PID
 * - Configuración de sensores y motores
 * - Constantes del sistema
 *
 * HARDWARE UTILIZADO:
 * - Microcontrolador: ESP32-S3 WROOM (FREENOVE)
 * - Sensores Array Lejano: 5x HW-511 individuales
 * - Sensores Array Cercano: 1x TCRT5000 módulo 5-en-1
 * - Puente H: L298N
 *
 * PLACA ESP32-S3 WROOM FREENOVE:
 * - Chip: ESP32-S3-WROOM-1
 * - PSRAM: 8MB integrado (GPIO35-37 ocupados)
 * - USB: Nativo (GPIO19-20 reservados)
 * - LED: WS2812 onboard en GPIO48
 * - Compatible con Arduino IDE: "ESP32S3 Dev Module"
 *
 * Autor: [Tu Nombre]
 * Fecha: 2025-10-29
 * Versión: 1.2.0 (ESP32-S3 WROOM FREENOVE)
 ******************************************************************************/

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/*******************************************************************************
 * CONFIGURACIÓN DE PINES - MOTORES L298N
 *
 * ESP32-S3 Pins usados (compatibles con PWM y GPIO estándar)
 ******************************************************************************/

// Motor Derecho FÍSICO → Lado derecho del robot (mirando desde atrás)
// NOTA: Conectado a OUT1/OUT2 del L298N (ENA/IN1/IN2)
#define MOTOR_DER_ENA   12    // GPIO12 - PWM (conectar a ENA del L298N)
#define MOTOR_DER_IN1   11    // GPIO11 - Control dirección (conectar a IN1 del L298N)
#define MOTOR_DER_IN2   18    // GPIO18 - Control dirección (conectar a IN2 del L298N)

// Motor Izquierdo FÍSICO → Lado izquierdo del robot (mirando desde atrás)
// NOTA: Conectado a OUT3/OUT4 del L298N (ENB/IN3/IN4)
#define MOTOR_IZQ_ENB   13    // GPIO13 - PWM (conectar a ENB del L298N)
#define MOTOR_IZQ_IN3   14    // GPIO14 - Control dirección (conectar a IN3 del L298N)
#define MOTOR_IZQ_IN4   21    // GPIO21 - Control dirección (conectar a IN4 del L298N)

// Canales PWM del ESP32 (0-15 disponibles)
#define PWM_CHANNEL_DER 0     // Canal PWM para motor derecho
#define PWM_CHANNEL_IZQ 1     // Canal PWM para motor izquierdo

// Configuración PWM
#define PWM_FREQUENCY   5000  // Frecuencia PWM en Hz (5kHz es óptimo para motores DC)
#define PWM_RESOLUTION  8     // Resolución PWM en bits (8 bits = 0-255)

/*******************************************************************************
 * CONFIGURACIÓN DE BOTONES - Control de usuario
 ******************************************************************************/

// Pines para botones de control (con pull-up interno)
#define BTN_PAUSE_RESUME    0     // GPIO0 - Botón para pausar/reanudar (BOOT button)
#define BTN_MODE_CHANGE     47    // GPIO47 - Botón para cambiar modo de operación
#define BTN_EMERGENCY_STOP  48    // GPIO48 - Botón de parada de emergencia (LED onboard)

// Configuración de debounce para botones
#define DEBOUNCE_DELAY      50    // Tiempo de debounce en milisegundos

// Estados de los botones (con pull-up, LOW = presionado)
#define BTN_PRESSED         LOW
#define BTN_RELEASED        HIGH

/*******************************************************************************
 * CONFIGURACIÓN DE PINES - SENSORES IR
 *
 * Configuración para un único array de 3 a 5 sensores analógicos.
 * El sistema está diseñado para ser flexible, simplemente ajustando NUM_SENSORES
 * y la definición de los pines y pesos correspondientes.
 ******************************************************************************/

// Cantidad de sensores que se están utilizando en el array.
// ¡Asegúrate de que los pines y pesos coincidan con este número!
#define NUM_SENSORES 5

// Asignación de pines para los sensores
// Configuración actualizada: GPIO 6-5-4-8-7 (orden físico: IZQ → CENTRO → DER)
#if NUM_SENSORES == 3
    #define SENSOR_1_PIN 3 // Izquierdo
    #define SENSOR_2_PIN 4 // Centro
    #define SENSOR_3_PIN 5 // Derecho
    const int8_t PESOS_SENSORES[NUM_SENSORES] = {-1, 0, 1}; // Pesos para 3 sensores
#elif NUM_SENSORES == 5
    #define SENSOR_1_PIN 6 // GPIO6 - Izquierda +2 (extremo izquierdo)
    #define SENSOR_2_PIN 5 // GPIO5 - Izquierda +1
    #define SENSOR_3_PIN 4 // GPIO4 - Centro
    #define SENSOR_4_PIN 8 // GPIO8 - Derecha +1
    #define SENSOR_5_PIN 7 // GPIO7 - Derecha +2 (extremo derecho)
    const int8_t PESOS_SENSORES[NUM_SENSORES] = {-2, -1, 0, 1, 2}; // Pesos para 5 sensores
#else
    #error "NUM_SENSORES debe ser 3 o 5. Por favor, ajusta la configuración."
#endif

// NOTAS IMPORTANTES SOBRE PINES EN ESP32-S3:
// - ADC1 (canales 0-9) corresponde a los GPIOs 1-10. Es ideal para lecturas estables.
// - ADC2 (canales 0-9) corresponde a los GPIOs 11-20. Se comparte con el WiFi y puede ser menos estable.
// - Se recomienda usar los pines del ADC1 (GPIO 1-10) para los sensores.

/*******************************************************************************
 * PARÁMETROS DE CALIBRACIÓN DE SENSORES
 ******************************************************************************/

// Rango del conversor Analógico-Digital (ADC) del ESP32-S3
#define ADC_RESOLUTION      12    // 12 bits = 0-4095
#define ADC_MAX_VALUE       4095  // Valor máximo del ADC

// Tiempo que dura el proceso de calibración automática al inicio (en milisegundos)
#define TIEMPO_CALIBRACION  8000  // 8 segundos


/*******************************************************************************
 * PARÁMETROS DE CONTROL PID
 *
 * El control PID se adapta según la curvatura detectada:
 * - RECTA: Control suave y estable
 * - CURVA_SUAVE: Mayor respuesta proporcional
 * - CURVA_CERRADA: Máxima agresividad, sin integral (evita wind-up)
 *
 * Ecuación PID:
 * u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt
 *
 * Donde:
 * - Kp: Ganancia Proporcional (respuesta inmediata al error)
 * - Ki: Ganancia Integral (corrige error acumulado)
 * - Kd: Ganancia Derivativa (anticipa cambios, reduce oscilación)
 ******************************************************************************/

// Estructura para parámetros PID
struct PIDParams {
    float Kp;  // Ganancia Proporcional
    float Ki;  // Ganancia Integral
    float Kd;  // Ganancia Derivativa
};

// Parámetros PID optimizados para 5 sensores
// Con mejor resolución espacial, se pueden usar parámetros más suaves
// Valores de sensores: BLANCO ~100 ADC, NEGRO ~2000 ADC
// Rango de error: -200 a +200 (con pesos -2, -1, 0, 1, 2)

const PIDParams PID_RECTA = {
    1.2,   // Kp - Respuesta proporcional suave (5 sensores dan mejor precisión)
    0.01,  // Ki - Corrección integral muy pequeña para deriva lenta
    0.8    // Kd - Amortiguación para evitar oscilaciones
};

const PIDParams PID_CURVA_SUAVE = {
    1.8,   // Kp - Mayor respuesta para curvas suaves
    0.02,  // Ki - Integral ligeramente mayor en curvas
    1.0    // Kd - Mayor amortiguación
};

const PIDParams PID_CURVA_CERRADA = {
    2.5,   // Kp - Respuesta agresiva para curvas cerradas
    0.0,   // Ki - Sin integral para evitar wind-up
    1.2    // Kd - Fuerte amortiguación para estabilidad máxima
};

/*******************************************************************************
 * PARÁMETROS DE VELOCIDAD Y MOTORES
 ******************************************************************************/

// Velocidades (escala 0-255 PWM lógico, mapeado a 40%-100% PWM real)
// NOTA: Con 12V de alimentación y mapeo automático al rango útil
// AJUSTADO PARA 5 SENSORES: Velocidades conservadoras para pruebas iniciales
#define VELOCIDAD_BASE      120   // Velocidad base en recta (0-255 lógico) - Muy conservadora
#define VELOCIDAD_MIN       30    // Velocidad mínima útil (será mapeada a ~40% PWM)
#define VELOCIDAD_MAX       255   // Velocidad máxima (mapeada a 100% PWM)
#define VELOCIDAD_CURVA     90    // Velocidad reducida en curvas cerradas - Extra conservadora

// Rango efectivo de PWM físico (elimina zona muerta 0-40%)
#define PWM_MIN_EFECTIVO    102   // 40% de 255 = punto de arranque real de motores
#define PWM_MAX_EFECTIVO    255   // 100% máximo

// Factor de compensación para equalizar motores
// AJUSTE FINO: Si un motor es más lento, aumentar su factor (>1.0)
//              Si un motor es más rápido, reducir su factor (<1.0)
// ✅ VALORES CALIBRADOS con test_motores.ino
#define FACTOR_MOTOR_DERECHO   1.00   // Factor para motor derecho (baseline)
#define FACTOR_MOTOR_IZQUIERDO 1.13   // Factor para motor izquierdo (+13% compensación)
                                      // Calibrado: Robot va recto sin desviación

// Límites de corrección PID
#define CORRECCION_MAX      100   // Máxima corrección que puede aplicar el PID

// Factor de reducción de velocidad según curvatura
#define FACTOR_VEL_CURVA_SUAVE    0.85  // Reducir a 85% en curvas suaves
#define FACTOR_VEL_CURVA_CERRADA  0.60  // Reducir a 60% en curvas cerradas

/*******************************************************************************
 * PARÁMETROS DE SENSORES
 ******************************************************************************/

// Umbrales de detección (valores ADC de 0-4095 en ESP32)
#define UMBRAL_LINEA_MIN    500   // Valor mínimo para considerar que hay línea
#define UMBRAL_LINEA_MAX    3000  // Valor máximo para línea blanca

// Parámetros de calibración
#define TIEMPO_CALIBRACION  8000  // Tiempo de calibración en milisegundos (8 segundos)
#define MUESTRAS_CALIBRACION 100  // Número de muestras para calibración

// Resolución ADC del ESP32
#define ADC_RESOLUTION      12    // 12 bits = 0-4095
#define ADC_MAX_VALUE       4095  // Valor máximo del ADC

/*******************************************************************************
 * PARÁMETROS DE FUSIÓN DE SENSORES
 *
 * El sistema fusiona las lecturas de ambos arrays de sensores con pesos
 * dinámicos según la velocidad:
 *
 * - Alta velocidad: Mayor peso a sensores lejanos (anticipación)
 * - Baja velocidad: Mayor peso a sensores cercanos (precisión)
 ******************************************************************************/

// Umbrales de velocidad para cambio de estrategia
#define UMBRAL_ALTA_VELOCIDAD   150   // PWM > 150 se considera alta velocidad
#define UMBRAL_BAJA_VELOCIDAD   100   // PWM < 100 se considera baja velocidad

// Pesos de fusión para ALTA velocidad (más anticipación)
#define PESO_CERCANO_ALTA_VEL   0.3   // 30% peso a sensores cercanos
#define PESO_LEJANO_ALTA_VEL    0.7   // 70% peso a sensores lejanos

// Pesos de fusión para BAJA velocidad (más precisión)
#define PESO_CERCANO_BAJA_VEL   0.6   // 60% peso a sensores cercanos
#define PESO_LEJANO_BAJA_VEL    0.4   // 40% peso a sensores lejanos

// Pesos de fusión para velocidad MEDIA (balance)
#define PESO_CERCANO_MEDIA_VEL  0.5   // 50% peso a sensores cercanos
#define PESO_LEJANO_MEDIA_VEL   0.5   // 50% peso a sensores lejanos

/*******************************************************************************
 * PARÁMETROS DE DETECCIÓN DE CURVATURA
 *
 * La curvatura se calcula como la diferencia absoluta entre los errores
 * de los arrays lejano y cercano:
 *
 * curvatura = |error_lejano - error_cercano|
 ******************************************************************************/

#define UMBRAL_CURVA_SUAVE      50    // Si curvatura > 50: curva suave
#define UMBRAL_CURVA_CERRADA    150   // Si curvatura > 150: curva cerrada

/*******************************************************************************
 * PARÁMETROS DE RECUPERACIÓN DE LÍNEA
 ******************************************************************************/

// Tiempos de recuperación
#define TIMEOUT_PERDIDA_LINEA   500   // Tiempo antes de entrar en modo búsqueda (ms)
#define TIMEOUT_BUSQUEDA        2000  // Tiempo máximo de búsqueda antes de detenerse (ms)

// Estrategia de búsqueda
#define VELOCIDAD_BUSQUEDA      120   // Velocidad durante búsqueda
#define ANGULO_BUSQUEDA         30    // Grados de giro durante búsqueda

/*******************************************************************************
 * PARÁMETROS DE TELEMETRÍA
 ******************************************************************************/

#define BAUDRATE                115200  // Velocidad del puerto serial
#define INTERVALO_TELEMETRIA    500     // Intervalo de envío de telemetría (ms)
#define TELEMETRIA_VERBOSE      true    // Mostrar telemetría detallada

/*******************************************************************************
 * ESTADOS DEL ROBOT
 ******************************************************************************/

enum EstadoRobot {
    CALIBRANDO,          // Calibrando sensores
    SIGUIENDO_LINEA,     // Siguiendo la línea normalmente
    PERDIDA_LINEA,       // Línea perdida temporalmente (< 500ms)
    BUSCANDO_LINEA,      // Buscando activamente la línea (girando)
    PAUSADO,             // Robot pausado (motores detenidos, esperando ajustes)
    CONFIGURACION,       // Modo de configuración interactiva
    DETENIDO,            // Robot detenido (error o fin de pista)
    DIAGNOSTICO          // Modo diagnóstico de hardware
};

/*******************************************************************************
 * BANDERAS DE CONTROL
 ******************************************************************************/

// Banderas volátiles para interrupciones
volatile bool flagPauseResume = false;       // Bandera para pausar/reanudar
volatile bool flagModeChange = false;        // Bandera para cambiar modo
volatile bool flagEmergencyStop = false;     // Bandera de parada de emergencia
volatile bool flagConfigChanged = false;     // Bandera de configuración modificada

// Banderas de estado
bool robotPausado = false;                   // Estado de pausa
bool permitirAjustes = false;                // Permitir ajustes en tiempo real
bool guardarConfigPendiente = false;         // Guardar configuración pendiente

/*******************************************************************************
 * CONFIGURACIÓN DE DEPURACIÓN
 ******************************************************************************/

// Habilitar/deshabilitar módulos de debug
#define DEBUG_SENSORES      false  // Mostrar lecturas de sensores
#define DEBUG_PID           false  // Mostrar cálculos PID
#define DEBUG_MOTORES       false  // Mostrar comandos a motores
#define DEBUG_ESTADOS       true   // Mostrar cambios de estado
#define DEBUG_FUSION        false  // Mostrar fusión de sensores

// Macro para debug condicional
#define DEBUG_PRINT(modulo, ...) if(modulo) { Serial.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(modulo, ...) if(modulo) { Serial.println(__VA_ARGS__); }

/*******************************************************************************
 * VALIDACIÓN DE CONFIGURACIÓN
 ******************************************************************************/

// Verificar que los valores de velocidad sean coherentes
#if VELOCIDAD_MIN > VELOCIDAD_BASE
    #error "VELOCIDAD_MIN no puede ser mayor que VELOCIDAD_BASE"
#endif

#if VELOCIDAD_BASE > VELOCIDAD_MAX
    #error "VELOCIDAD_BASE no puede ser mayor que VELOCIDAD_MAX"
#endif

// Verificar que los pesos de fusión sumen 1.0
// NOTA: No se puede usar #if con flotantes, esta validación se hace en tiempo de compilación
// pero podría moverse a tiempo de ejecución si se necesita validación estricta
// #if (PESO_CERCANO_ALTA_VEL + PESO_LEJANO_ALTA_VEL) != 1.0
//     #warning "Los pesos de fusión para alta velocidad no suman 1.0"
// #endif

/*******************************************************************************
 * MAPA DE PINES - RESUMEN
 *
 * Para referencia rápida durante el montaje:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  GPIO  │  Función           │  Conexión                         │
 * ├────────┼────────────────────┼───────────────────────────────────┤
 * │  27    │  Motor Der PWM     │  L298N ENA                        │
 * │  26    │  Motor Der IN1     │  L298N IN1                        │
 * │  25    │  Motor Der IN2     │  L298N IN2                        │
 * │  33    │  Motor Izq PWM     │  L298N ENB                        │
 * │  32    │  Motor Izq IN3     │  L298N IN3                        │
 * │  14    │  Motor Izq IN4     │  L298N IN4                        │
 * ├────────┼────────────────────┼───────────────────────────────────┤
 * │  36    │  IR Lejano 0       │  Sensor IR Extremo Izq (15cm)     │
 * │  39    │  IR Lejano 1       │  Sensor IR Interior Izq (15cm)    │
 * │  34    │  IR Lejano 2       │  Sensor IR Centro (15cm)          │
 * │  35    │  IR Lejano 3       │  Sensor IR Interior Der (15cm)    │
 * │  4     │  IR Lejano 4       │  Sensor IR Extremo Der (15cm)     │
 * ├────────┼────────────────────┼───────────────────────────────────┤
 * │  15    │  IR Cercano 5      │  Sensor IR Extremo Izq (5cm)      │
 * │  2     │  IR Cercano 6      │  Sensor IR Interior Izq (5cm)     │
 * │  0     │  IR Cercano 7      │  Sensor IR Centro (5cm) ⚠️ BOOT   │
 * │  12    │  IR Cercano 8      │  Sensor IR Interior Der (5cm)     │
 * │  13    │  IR Cercano 9      │  Sensor IR Extremo Der (5cm)      │
 * └────────┴────────────────────┴───────────────────────────────────┘
 *
 * ⚠️ ADVERTENCIAS:
 * - GPIO0: Pin de BOOT. Desconectar sensor durante programación si hay problemas
 * - GPIO34-39: Solo INPUT (ADC1), no pueden ser OUTPUT
 * - GPIO12: Puede interferir con flash en algunos módulos ESP32
 *
 * 💡 RECOMENDACIONES:
 * - Usar GPIO34-39 para sensores (solo lectura)
 * - Usar GPIO con PWM para motores
 * - Dejar GPIO1 y GPIO3 libres (TX/RX del serial)
 * - Conectar GND común entre ESP32, L298N y sensores
 ******************************************************************************/

#endif // CONFIG_H
