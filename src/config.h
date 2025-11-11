/*******************************************************************************
 * CONFIG.H - Configuración de Pines y Parámetros
 *
 * Este archivo centraliza toda la configuración del robot seguidor de línea:
 * - Definición de pines GPIO del ESP32-S3
 * - Parámetros de control PID adaptativo
 * - Configuración de sensores y motores
 * - Constantes del sistema
 *
 * HARDWARE UTILIZADO:
 * - Microcontrolador: ESP32-S3 WROOM (FREENOVE)
 * - Sensores: 5x HW-511 individuales (GPIO 6, 5, 4, 8, 7)
 * - Puente H: L298N para control de motores DC
 * - Motores: 2x DC con reductora
 *
 * PLACA ESP32-S3 WROOM FREENOVE:
 * - Chip: ESP32-S3-WROOM-1
 * - PSRAM: 8MB integrado (GPIO35-37 ocupados)
 * - USB: Nativo (GPIO19-20 reservados)
 * - LED: WS2812 onboard en GPIO48
 * - Compatible con Arduino IDE: "ESP32S3 Dev Module"
 *
 * Autor: LUCHIN-OPRESORCL
 * Fecha: 2025-11-07
 * Versión: 2.0.0
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
    // Usando solo sensores extremos y centro (sin sensores 2 y 4)
    #define SENSOR_1_PIN 6 // GPIO6 - Extremo IZQUIERDO (antes sensor 1)
    #define SENSOR_2_PIN 4 // GPIO4 - CENTRO (antes sensor 3)
    #define SENSOR_3_PIN 7 // GPIO7 - Extremo DERECHO (antes sensor 5)
    const int8_t PESOS_SENSORES[NUM_SENSORES] = {-4, 0, 4}; // Pesos: extremos ±4, centro 0
#elif NUM_SENSORES == 5
    #define SENSOR_1_PIN 6 // GPIO6 - Izquierda -4 (extremo izquierdo)
    #define SENSOR_2_PIN 5 // GPIO5 - Izquierda -1
    #define SENSOR_3_PIN 4 // GPIO4 - Centro 0
    #define SENSOR_4_PIN 8 // GPIO8 - Derecha +1
    #define SENSOR_5_PIN 7 // GPIO7 - Derecha +4 (extremo derecho)
    const int8_t PESOS_SENSORES[NUM_SENSORES] = {-5, -1, 0, 1, 5}; // Pesos exponenciales: extremos +100%
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

// Umbral de detección de línea negra (en escala normalizada 0-100)
// Valores típicos: BLANCO ~100 ADC (~2 normalizado), NEGRO ~2000 ADC (~95 normalizado)
// Para detectar negro > 1500 ADC, usar umbral ~74
#define UMBRAL_DETECCION_LINEA  74  // Valor normalizado para considerar línea negra detectada


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
// Usando todos los sensores (GPIOs 6, 5, 4, 8, 7)
// Valores de sensores: BLANCO ~100 ADC, NEGRO ~2000 ADC
// Rango de error: -400 a +400 (con pesos: -5, -1, 0, 1, 5)
//                 Negativo = línea a la IZQ, Positivo = línea a la DER
//                 Los extremos tienen peso exponencial para mejor respuesta en curvas
// NOTA: Estos parámetros son modificables en runtime con el comando 'p'

// SISTEMA SIMPLIFICADO: SOLO 2 MODOS PID
// Valores por defecto para cada modo (se restauran al reiniciar)
#define PID_RECTA_DEFAULT_KP    1.0
#define PID_RECTA_DEFAULT_KI    0.005
#define PID_RECTA_DEFAULT_KD    0.5

#define PID_CERRADA_DEFAULT_KP  2.5
#define PID_CERRADA_DEFAULT_KI  0.0
#define PID_CERRADA_DEFAULT_KD  1.2

// Variables globales modificables (inicializadas en main.cpp)
extern PIDParams PID_RECTA;
extern PIDParams PID_CURVA_CERRADA;
extern bool pidAdaptativoActivo;  // true = modo adaptativo, false = modo manual

/*******************************************************************************
 * PARÁMETROS DE VELOCIDAD Y MOTORES
 ******************************************************************************/

// Velocidades (escala 0-255 PWM lógico, mapeado a 40%-100% PWM real)
// NOTA: Con 12V de alimentación y mapeo automático al rango útil
// AJUSTADO PARA 5 SENSORES: Velocidades conservadoras para pruebas iniciales
#define VELOCIDAD_BASE      120   // Velocidad base en recta (0-255 lógico) - Muy conservadora
#define VELOCIDAD_MIN       30    // Velocidad mínima útil (será mapeada a ~40% PWM)
#define VELOCIDAD_MAX       255   // Velocidad máxima (mapeada a 100% PWM)
#define VELOCIDAD_CURVA     120    // Velocidad reducida en curvas cerradas - Extra conservadora

// Rango efectivo de PWM físico (elimina zona muerta 0-40%)
#define PWM_MIN_EFECTIVO    130   // 40% de 255 = punto de arranque real de motores
#define PWM_MAX_EFECTIVO    255   // 100% máximo

/*******************************************************************************
 * FACTORES DE COMPENSACIÓN DE MOTORES
 *
 * Debido a tolerancias de fabricación, los motores DC nunca son idénticos.
 * Un motor puede girar más rápido que el otro al mismo PWM, causando que
 * el robot se desvíe en línea recta.
 *
 * METODOLOGÍA DE CALIBRACIÓN:
 * ────────────────────────────
 * 1. PREPARACIÓN:
 *    - Marcar una línea recta de 3 metros en el suelo
 *    - Cargar sketch: pruebas/test_motores.ino
 *    - Asegurar batería completamente cargada (12V)
 *
 * 2. PRUEBA INICIAL:
 *    - Ejecutar test con velocidad 150 PWM
 *    - Observar hacia qué lado se desvía el robot
 *    - Medir desviación lateral al final de 3m
 *
 * 3. IDENTIFICAR MOTOR MÁS RÁPIDO:
 *    - Si se desvía a la DERECHA → motor IZQUIERDO más rápido
 *    - Si se desvía a la IZQUIERDA → motor DERECHO más rápido
 *
 * 4. AJUSTE ITERATIVO:
 *    - Aumentar factor del motor MÁS LENTO en incrementos de 0.05
 *    - Repetir prueba hasta lograr trayectoria recta
 *    - Validar con velocidades 100, 150, 200 PWM
 *
 * 5. CRITERIOS DE ACEPTACIÓN:
 *    ✓ Desviación lateral < 5cm en 3 metros
 *    ✓ Desviación angular < 2° en trayectoria completa
 *    ✓ Comportamiento consistente en diferentes velocidades
 *
 * VALORES ACTUALES (Calibrados: 2025-11-07):
 * ────────────────────────────────────────────
 * - Motor Derecho:   Factor 1.00 (baseline, sin compensación)
 * - Motor Izquierdo: Factor 1.13 (+13% compensación)
 *
 * INTERPRETACIÓN:
 *   El motor izquierdo es físicamente más débil que el derecho.
 *   Requiere 13% más PWM para alcanzar las mismas RPM.
 *
 * PRUEBAS DE VALIDACIÓN:
 *   ✓ Robot avanza 3m en línea recta con desviación < 5cm
 *   ✓ Desviación angular medida: 1.2° (objetivo: <2°)
 *   ✓ Comportamiento estable en rango 100-200 PWM
 *
 * NOTA: Si cambias los motores, debes RE-CALIBRAR estos valores.
 ******************************************************************************/
#define FACTOR_MOTOR_DERECHO   1.00   // Baseline (motor de referencia)
#define FACTOR_MOTOR_IZQUIERDO 1.08   // +08% compensación (motor más débil)

// Límites de corrección PID
#define CORRECCION_MAX      100   // Máxima corrección que puede aplicar el PID (valor absoluto)
#define CORRECCION_MAX_PORCENTAJE  0.8  // Corrección máxima como % de velocidad actual (80%)
                                        // Garantiza que ambas ruedas siempre giren hacia adelante

// Banda muerta (deadband) para errores muy pequeños
// Si el error es menor que esto, se ignora (evita zigzagueo por ruido)
#define ERROR_DEADBAND      5     // Ignorar errores menores a ±5

// Factor de reducción de velocidad según curvatura (SIMPLIFICADO - SOLO CURVA CERRADA)
#define FACTOR_VEL_CURVA_CERRADA  0.60  // Reducir a 60% en curvas cerradas

// Amplificador de corrección para errores grandes (transición gradual)
// En lugar de detener una rueda, amplificamos la corrección del PID
// NOTA: Ajustado para nuevo rango de error (-400 a +400)
#define UMBRAL_AMPLIFICACION_MIN  200   // Error a partir del cual empieza amplificación
#define UMBRAL_AMPLIFICACION_MAX  320   // Error donde alcanza amplificación máxima
#define FACTOR_AMPLIFICACION_MIN  1.0   // Sin amplificación (error < 200)
#define FACTOR_AMPLIFICACION_MAX  1.8   // Amplificación máxima (error > 320) - 80% más corrección

// Giro en pivote para curvas extremadamente cerradas (horquillas 180°)
// Cuando solo los sensores extremos detectan la línea, activar pivote
#define UMBRAL_GIRO_CRITICO       350   // Error crítico: solo sensores extremos activos (87.5% del máximo)
#define VELOCIDAD_PIVOTE_INTERIOR 10    // Velocidad rueda interior: 15% = 38 PWM (pivote asistido, reduce fricción)
#define VELOCIDAD_PIVOTE_EXTERIOR 90    // Velocidad rueda exterior: 85% = 217 PWM (giro agresivo controlado)

/*******************************************************************************
 * PARÁMETROS DE SENSORES
 ******************************************************************************/

// Umbrales de detección (valores ADC de 0-4095 en ESP32-S3)
#define UMBRAL_LINEA_MIN    500   // Valor mínimo para considerar que hay línea
#define UMBRAL_LINEA_MAX    3000  // Valor máximo para línea blanca

// Parámetros de calibración adicionales
#define MUESTRAS_CALIBRACION 100  // Número de muestras para calibración

/*******************************************************************************
 * PARÁMETROS DE DETECCIÓN DE CURVATURA (SIMPLIFICADO - 2 MODOS)
 *
 * La curvatura se calcula combinando dos factores:
 * curvatura = abs(error) * PESO_ERROR_CURVATURA + tasaCambio * PESO_TASA_CAMBIO
 *
 * Donde:
 * - abs(error): Magnitud de desviación (-400 a +400)
 * - tasaCambio: Velocidad con que cambia el error (anticipación)
 *
 * SISTEMA SIMPLIFICADO: Solo 2 modos (RECTA y CURVA_CERRADA)
 ******************************************************************************/

// Pesos para cálculo de curvatura
#define PESO_ERROR_CURVATURA       0.7    // 70% peso al error absoluto
#define PESO_TASA_CAMBIO_CURVATURA 0.3    // 30% peso a la tasa de cambio

// Umbral de curvatura para cambio de modo PID (SIMPLIFICADO)
#define UMBRAL_CURVA_CERRADA    140   // Si curvatura ≥ 140: CURVA_CERRADA, sino: RECTA

// Factor de filtro exponencial para el error
#define ALPHA_FILTRO_ERROR      0.7   // 70% nuevo, 30% histórico (suaviza ruido)

/*******************************************************************************
 * PARÁMETROS DE RECUPERACIÓN DE LÍNEA
 ******************************************************************************/

// Tiempos de recuperación (estrategia de 3 fases)
#define TIMEOUT_PERDIDA_LINEA   1500  // Fase 1: Tolerancia inicial - mantiene dirección (ms)
#define TIMEOUT_RETROCESO       2500  // Fase 2: Tiempo de retroceso inteligente (ms) [1500ms de retroceso]
#define TIMEOUT_BUSQUEDA        3500  // Fase 3: Búsqueda activa antes de detenerse (ms) [1000ms de búsqueda]

// Estrategia de retroceso inteligente (Fase 2)
// NOTA: Ahora usa velocidadBase configurada dinámicamente (comando 'v'), no valor fijo
#define FACTOR_GIRO_RETROCESO   0.6   // Factor de giro: rueda interior al 60% de la exterior

// Estrategia de búsqueda activa (Fase 3)
#define VELOCIDAD_BUSQUEDA      120   // Velocidad durante búsqueda
#define ANGULO_BUSQUEDA         30    // Grados de giro durante búsqueda

/*******************************************************************************
 * PARÁMETROS DE TELEMETRÍA Y CONTROL DE CICLO
 ******************************************************************************/

#define BAUDRATE                115200  // Velocidad del puerto serial
#define INTERVALO_TELEMETRIA    500     // Intervalo de envío de telemetría (ms)
#define TELEMETRIA_VERBOSE      true    // Mostrar telemetría detallada

// Delays del ciclo principal
#define DELAY_CICLO_CONTROL     5       // Delay del loop principal (ms) = ~200Hz frecuencia de control
#define DELAY_TEST_MOTOR_PASO   2000    // Duración de cada paso en test de motores (ms)
#define DELAY_TEST_ENTRE_PASOS  500     // Pausa entre pasos del test de motores (ms)

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

/*******************************************************************************
 * MAPA DE PINES - ESP32-S3 WROOM FREENOVE (CONFIGURACIÓN ACTUAL)
 *
 * Para referencia rápida durante el montaje:
 *
 * ┌─────────────────────────────────────────────────────────────────┐
 * │  GPIO  │  Función           │  Conexión                         │
 * ├────────┼────────────────────┼───────────────────────────────────┤
 * │  MOTORES L298N                                                   │
 * ├────────┼────────────────────┼───────────────────────────────────┤
 * │  12    │  Motor Der PWM     │  L298N ENA                        │
 * │  11    │  Motor Der IN1     │  L298N IN1                        │
 * │  18    │  Motor Der IN2     │  L298N IN2                        │
 * │  13    │  Motor Izq PWM     │  L298N ENB                        │
 * │  14    │  Motor Izq IN3     │  L298N IN3                        │
 * │  21    │  Motor Izq IN4     │  L298N IN4                        │
 * ├────────┼────────────────────┼───────────────────────────────────┤
 * │  SENSORES IR (5x HW-511)                                         │
 * ├────────┼────────────────────┼───────────────────────────────────┤
 * │  6     │  Sensor 1 (S1)     │  Extremo Izquierdo (Peso: -3)     │
 * │  5     │  Sensor 2 (S2)     │  Izquierda (Peso: -1)             │
 * │  4     │  Sensor 3 (S3)     │  Centro (Peso: 0)                 │
 * │  8     │  Sensor 4 (S4)     │  Derecha (Peso: +1)               │
 * │  7     │  Sensor 5 (S5)     │  Extremo Derecho (Peso: +3)       │
 * ├────────┼────────────────────┼───────────────────────────────────┤
 * │  BOTONES DE CONTROL                                              │
 * ├────────┼────────────────────┼───────────────────────────────────┤
 * │  0     │  Pausar/Reanudar   │  BOOT button (Pull-up interno)    │
 * │  47    │  Cambiar Modo      │  Pull-up interno                  │
 * │  48    │  Parada Emergencia │  LED onboard (Pull-up interno)    │
 * └────────┴────────────────────┴───────────────────────────────────┘
 *
 * ⚠️ ADVERTENCIAS ESP32-S3:
 * - GPIO0: Pin de BOOT. No usar durante programación
 * - GPIO19-20: USB nativo, reservados
 * - GPIO35-37: PSRAM, ocupados por hardware
 * - GPIO48: LED WS2812 onboard
 *
 * 💡 RECOMENDACIONES:
 * - ADC1 (GPIO1-10): Ideal para sensores analógicos (lecturas estables)
 * - ADC2 (GPIO11-20): Compartido con WiFi, puede ser inestable
 * - PWM: Disponible en cualquier GPIO de salida
 * - Conectar GND común entre ESP32-S3, L298N y sensores
 * - Alimentación L298N: 12V externa (NO desde ESP32)
 ******************************************************************************/

#endif // CONFIG_H
