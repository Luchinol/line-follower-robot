/*******************************************************************************
 * MAIN.CPP - Carrito Seguidor de Línea ESP32-S3
 *
 * Sistema de seguimiento de línea autónomo con:
 * - Array único de 5 sensores IR HW-511 con pesos exponenciales
 * - Control PID adaptativo según curvatura detectada
 * - Amplificación gradual de corrección en errores grandes
 * - Máquina de estados para gestión de comportamiento
 * - Recuperación automática ante pérdida de línea
 * - Telemetría y comandos seriales en tiempo real
 *
 * ═══════════════════════════════════════════════════════════════════════════
 * DIAGRAMA DE MÁQUINA DE ESTADOS
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *                        ┌──────────────┐
 *            POWER ON ──→│  CALIBRANDO  │
 *                        └──────┬───────┘
 *                               │ (auto, 8seg)
 *                               ↓
 *                        ┌──────────────┐
 *                ┌──────→│ SIGUIENDO_   │←──────┐
 *                │       │    LINEA     │       │
 *                │       └──┬────────┬──┘       │
 *                │          │        │          │
 *         (línea │          │        │ (línea   │
 *     recuperada)│          │        │ perdida) │
 *                │          ↓        ↓          │
 *         ┌──────┴─────┐ ┌──────────────┐      │
 *         │ BUSCANDO_  │ │  PERDIDA_    │      │
 *         │   LINEA    │←│   LINEA      │      │
 *         └──────┬─────┘ └──────────────┘      │
 *                │              (timeout 500ms) │
 *                │ (timeout 2s)                 │
 *                ↓                              │
 *         ┌─────────────┐               ┌──────┴────────┐
 *     ┌──│  DETENIDO   │               │   PAUSADO     │
 *     │  └─────────────┘               └───────────────┘
 *     │         ↑                               ↑  ↓
 *     │         │ (emergency)          (pause)  │  │ (start)
 *     │         │                               │  │
 *     │         └───────────────────────────────┘  │
 *     │                                            │
 *     │  ┌──────────────────────────────────────┐ │
 *     └─→│ CONFIGURACION / DIAGNOSTICO          │←┘
 *        └──────────────────────────────────────┘
 *           (comandos especiales: 'd', 'config')
 *
 * DESCRIPCIÓN DE ESTADOS:
 * ═════════════════════════
 *   CALIBRANDO      → Calibración automática de sensores (8 seg)
 *   SIGUIENDO_LINEA → Seguimiento normal con PID adaptativo
 *   PERDIDA_LINEA   → Línea perdida temporalmente (< 500ms)
 *   BUSCANDO_LINEA  → Búsqueda activa girando (max 2 seg)
 *   PAUSADO         → Robot en pausa (acepta comandos)
 *   CONFIGURACION   → Modo de configuración interactiva
 *   DETENIDO        → Robot detenido (seguridad)
 *   DIAGNOSTICO     → Test de hardware y sensores
 *
 * Hardware:
 *   - ESP32-S3 WROOM (FREENOVE)
 *   - 5x Sensores IR HW-511 (GPIO 6, 5, 4, 8, 7)
 *   - L298N (puente H para motores DC)
 *   - 2x Motores DC con reductora
 *
 * Autor: LUCHIN-OPRESORCL
 * Fecha: 2025-11-07
 * Versión: 2.0.0
 ******************************************************************************/

#include <Arduino.h>

// Incluir archivos de configuración y librerías
#include "config.h"
#include "sensores.h"
#include "motores.h"
#include "control_pid.h"
#include "nvs_config.h"

/*******************************************************************************
 * INSTANCIAS GLOBALES
 ******************************************************************************/

SensoresIR sensores;                        // Gestor de sensores IR
ControlMotores motores;                     // Control de motores
ControladorPID pid(                         // Controlador PID (inicialmente en modo RECTA)
    PID_RECTA.Kp,
    PID_RECTA.Ki,
    PID_RECTA.Kd
);
ConfiguracionNVS configNVS;                 // Gestor de configuración persistente

/*******************************************************************************
 * VARIABLES GLOBALES
 ******************************************************************************/

// Parámetros PID modificables en runtime (SIMPLIFICADO - 2 MODOS)
PIDParams PID_RECTA = {PID_RECTA_DEFAULT_KP, PID_RECTA_DEFAULT_KI, PID_RECTA_DEFAULT_KD};
PIDParams PID_CURVA_CERRADA = {PID_CERRADA_DEFAULT_KP, PID_CERRADA_DEFAULT_KI, PID_CERRADA_DEFAULT_KD};
bool pidAdaptativoActivo = true;  // Por defecto, el PID adaptativo está activo

// Estado actual del robot
EstadoRobot estadoActual = CALIBRANDO;
EstadoRobot estadoAnterior = CALIBRANDO;

// Velocidad base del robot (puede ajustarse dinámicamente)
uint8_t velocidadBase = VELOCIDAD_BASE;

// Variables de telemetría (actualizadas en cada ciclo de control)
int16_t errorActual = 0;         // Error filtrado actual para telemetría
uint16_t curvaturaActual = 0;    // Curvatura detectada para telemetría

// Tiempo de última detección de línea
unsigned long tiempoPerdidaLinea = 0;

// Memoria del último error significativo (para retroceso inteligente)
int16_t ultimoErrorSignificativo = 0;  // Guardamos el último error cuando |error| > 100

// Última corrección PID conocida antes de perder la línea
float ultimaCorreccionConocida = 0.0f;

// Dirección de búsqueda (true = derecha, false = izquierda)
bool direccionBusqueda = true;

// Telemetría
unsigned long tiempoUltimaTelemetria = 0;
unsigned long ciclosProcesamiento = 0;
unsigned long tiempoInicio = 0;

// Buffer de comando serial (tamaño fijo para evitar fragmentación de heap)
#define COMANDO_BUFFER_SIZE 64
char comandoBuffer[COMANDO_BUFFER_SIZE];
uint8_t comandoIndex = 0;

/*******************************************************************************
 * DECLARACIONES ADELANTADAS (Forward declarations)
 ******************************************************************************/
void estadoCalibrar();
void estadoSeguirLinea();
void estadoPerdidaLinea();
void estadoBuscarLinea();
void estadoPausado();
void estadoConfiguracion();
void estadoDetenido();
void estadoDiagnostico();
void cambiarEstado(EstadoRobot nuevoEstado);
void detenerYCambiarEstado(EstadoRobot nuevoEstado);
const char* nombreEstado(EstadoRobot estado);
void enviarTelemetria();
void procesarComandosSerial();
void ejecutarComando(String cmd);
void mostrarAyuda();
void mostrarMenuConfiguracion();
void procesarBanderas();
void inicializarBotones();

/*******************************************************************************
 * FUNCIONES DE INTERRUPCIÓN (ISR)
 ******************************************************************************/

// ISR para botón de pausa/reanudación
void IRAM_ATTR isrPauseResume() {
    static unsigned long ultimaInterrupcion = 0;
    unsigned long tiempoActual = millis();

    // Debounce simple
    if (tiempoActual - ultimaInterrupcion > DEBOUNCE_DELAY) {
        flagPauseResume = true;
        ultimaInterrupcion = tiempoActual;
    }
}

// ISR para botón de cambio de modo
void IRAM_ATTR isrModeChange() {
    static unsigned long ultimaInterrupcion = 0;
    unsigned long tiempoActual = millis();

    if (tiempoActual - ultimaInterrupcion > DEBOUNCE_DELAY) {
        flagModeChange = true;
        ultimaInterrupcion = tiempoActual;
    }
}

// ISR para parada de emergencia
void IRAM_ATTR isrEmergencyStop() {
    static unsigned long ultimaInterrupcion = 0;
    unsigned long tiempoActual = millis();

    if (tiempoActual - ultimaInterrupcion > DEBOUNCE_DELAY) {
        flagEmergencyStop = true;
        ultimaInterrupcion = tiempoActual;
    }
}

/*******************************************************************************
 * SETUP - Inicialización del sistema
 ******************************************************************************/
void setup() {
    // Inicializar comunicación serial
    Serial.begin(BAUDRATE);
    delay(500);

    Serial.println("\n\n");
    Serial.println("========================================");
    Serial.println("   CARRITO SEGUIDOR DE LÍNEA ESP32");
    Serial.println("========================================");
    Serial.println("Versión: 1.0.0");
    Serial.println("Autor: LUCHIN-OPRESORCL");
    Serial.println("Fecha: 2025-10-29");
    Serial.println("========================================\n");

    // Mostrar información del sistema
    Serial.println("INFORMACIÓN DEL SISTEMA:");
    Serial.print("  Chip: "); Serial.println(ESP.getChipModel());
    Serial.print("  Núcleos: "); Serial.println(ESP.getChipCores());
    Serial.print("  Frecuencia CPU: "); Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");
    Serial.print("  RAM libre: "); Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");
    Serial.println();

    // Inicializar módulos
    Serial.println("INICIALIZANDO MÓDULOS...\n");

    // 1. Inicializar sistema NVS y cargar configuración guardada
    configNVS.inicializar();

    // Intentar cargar configuración desde Flash
    float kp_cargado, ki_cargado, kd_cargado;
    uint8_t vel_cargada;

    if (configNVS.cargarConfiguracion(vel_cargada, kp_cargado, ki_cargado, kd_cargado)) {
        // Aplicar configuración cargada
        velocidadBase = vel_cargada;
        pid.setParametros(kp_cargado, ki_cargado, kd_cargado);
        Serial.println("  ✓ Configuración aplicada desde memoria Flash");
    } else {
        Serial.println("  ⓘ Usando valores por defecto de config.h");
    }
    delay(100);

    // 2. Inicializar botones e interrupciones
    inicializarBotones();
    delay(100);

    // 3. Inicializar sensores IR
    sensores.inicializar();
    delay(100);

    // 4. Inicializar motores
    motores.inicializar();
    delay(100);

    // 5. Inicializar PID (ya configurado con valores cargados o por defecto)
    pid.inicializar();
    delay(100);

    Serial.println("========================================");
    Serial.println("SISTEMA INICIALIZADO CORRECTAMENTE");
    Serial.println("========================================\n");

    // Iniciar calibración automática
    sensores.iniciarCalibracion();
    estadoActual = CALIBRANDO;
    tiempoInicio = millis();

    // Mostrar ayuda de comandos
    mostrarAyuda();
}

/*******************************************************************************
 * LOOP - Ciclo principal del robot
 ******************************************************************************/
void loop() {
    // Procesar banderas de interrupciones PRIMERO
    procesarBanderas();

    // Procesar comandos seriales
    procesarComandosSerial();

    // Máquina de estados principal
    switch (estadoActual) {
        case CALIBRANDO:
            estadoCalibrar();
            break;

        case SIGUIENDO_LINEA:
            estadoSeguirLinea();
            break;

        case PERDIDA_LINEA:
            estadoPerdidaLinea();
            break;

        case BUSCANDO_LINEA:
            estadoBuscarLinea();
            break;

        case PAUSADO:
            estadoPausado();
            break;

        case CONFIGURACION:
            estadoConfiguracion();
            break;

        case DETENIDO:
            estadoDetenido();
            break;

        case DIAGNOSTICO:
            estadoDiagnostico();
            break;
    }

    // Telemetría periódica
    if (millis() - tiempoUltimaTelemetria > INTERVALO_TELEMETRIA) {
        if (TELEMETRIA_VERBOSE && estadoActual == SIGUIENDO_LINEA) {
            enviarTelemetria();
        }
        tiempoUltimaTelemetria = millis();
    }

    // Incrementar contador de ciclos
    ciclosProcesamiento++;

    // Delay del ciclo de control para mantener frecuencia estable (~200 Hz)
    // Frecuencia mayor → mejor seguimiento en rectas (menos zigzagueo)
    delay(DELAY_CICLO_CONTROL);
}

/*******************************************************************************
 * ESTADO: CALIBRANDO
 *
 * Calibra los sensores IR durante TIEMPO_CALIBRACION segundos.
 * El robot debe moverse manualmente sobre blanco y negro.
 ******************************************************************************/
void estadoCalibrar() {
    if (sensores.actualizarCalibracion()) {
        // Calibración completada
        cambiarEstado(SIGUIENDO_LINEA);
        Serial.println("\n¡Calibración completada! Iniciando seguimiento...\n");
    }
}

/*******************************************************************************
 * ESTADO: SIGUIENDO_LINEA
 *
 * Estado principal: sigue la línea usando control PID adaptativo.
 ******************************************************************************/
void estadoSeguirLinea() {
    // 1. Leer los valores crudos de los sensores
    sensores.leer();

    // 2. Procesar los valores para obtener el error de posición
    int16_t error = sensores.procesar();

    // 3. Verificar si la línea está visible
    if (!sensores.isLineaDetectada()) {
        // Si no, cambiar al estado de pérdida de línea
        tiempoPerdidaLinea = millis();
        cambiarEstado(PERDIDA_LINEA);
        return;
    }

    /***************************************************************************
     * 4. FILTRO DE ERROR - Media Móvil Exponencial (EMA)
     *
     * Reduce zigzagueo causado por ruido en sensores sin agregar lag excesivo.
     *
     * FÓRMULA:
     *   errorFiltrado(n) = α × error(n) + (1-α) × errorFiltrado(n-1)
     *
     * Donde α = ALPHA_FILTRO_ERROR = 0.7
     *
     * CARACTERÍSTICAS:
     *   - Respuesta rápida: 70% del nuevo valor se incorpora inmediatamente
     *   - Suavizado: 30% de historia previa amortigua cambios bruscos
     *   - Lag mínimo: Adecuado para control en tiempo real
     *
     * DECISIÓN DE IMPLEMENTACIÓN:
     *   ✓ Aritmética FLOTANTE (float) en vez de entera:
     *     - ESP32-S3 tiene FPU hardware → operaciones float rápidas
     *     - Mayor precisión sin overhead significativo
     *     - Código más simple y mantenible
     *     - Alternativa entera requeriría escalado complejo (ej: *1000)
     *
     * RENDIMIENTO MEDIDO:
     *   - Tiempo de ejecución: ~2-3 µs en ESP32-S3 @ 240MHz
     *   - Overhead: Despreciable en ciclo de 5ms (0.06%)
     ***************************************************************************/
    static float errorFiltrado = 0;
    errorFiltrado = ALPHA_FILTRO_ERROR * error + (1.0 - ALPHA_FILTRO_ERROR) * errorFiltrado;

    // Aplicar banda muerta (deadband) para errores muy pequeños
    // Evita correcciones innecesarias por ruido cuando está casi centrado
    if (abs(errorFiltrado) < ERROR_DEADBAND) {
        errorFiltrado = 0;
    }

    /***************************************************************************
     * 5. DETECCIÓN DE CURVATURA (Algoritmo de Estimación Adaptativa)
     *
     * Este algoritmo combina dos factores para estimar la curvatura de la pista
     * y adaptar el comportamiento del PID en tiempo real:
     *
     * FACTOR 1: ERROR ABSOLUTO (70% de peso)
     *   - Magnitud de la desviación actual respecto al centro
     *   - Rango: 0 a 300 (con pesos exponenciales -3, -1, 0, +1, +3)
     *   - Mayor error → curva más cerrada o desviación significativa
     *
     * FACTOR 2: TASA DE CAMBIO (30% de peso)
     *   - Velocidad con que cambia el error (derivada aproximada)
     *   - Unidades: error/segundo
     *   - Permite ANTICIPACIÓN: detecta curvas antes de que el error sea grande
     *   - Ejemplo: Si error pasa de 50 a 150 en 0.1s → tasa = 1000
     *
     * FÓRMULA:
     *   curvatura = |errorFiltrado| × 0.7 + tasaCambio × 0.3
     *
     * UMBRALES DE DECISIÓN:
     *   - curvatura < 80:  RECTA → PID suave (Kp=1.0, Ki=0.005, Kd=0.5)
     *   - 80 ≤ curv < 140: CURVA_SUAVE → PID moderado (Kp=1.8, Ki=0.02, Kd=1.0)
     *   - curvatura ≥ 140: CURVA_CERRADA → PID agresivo (Kp=2.5, Ki=0.0, Kd=1.2)
     *
     * VENTAJAS:
     *   ✓ Anticipación: detecta curvas antes por la tasa de cambio
     *   ✓ Robustez: el error absoluto da contexto inmediato
     *   ✓ Suavidad: transición gradual entre modos PID
     ***************************************************************************/
    static int16_t errorAnterior = 0;
    static unsigned long tiempoAnterior = 0;

    unsigned long tiempoActual = millis();
    float dt = (tiempoActual - tiempoAnterior) / 1000.0;
    if (dt <= 0 || dt > 1.0) dt = 0.01; // Protección contra valores inválidos

    // Tasa de cambio del error (derivada aproximada)
    // Usamos errorFiltrado para evitar amplificar ruido
    float tasaCambio = abs(errorFiltrado - errorAnterior) / dt;

    // Curvatura estimada: combinación ponderada
    uint16_t curvatura = (uint16_t)(abs(errorFiltrado) * PESO_ERROR_CURVATURA +
                                     tasaCambio * PESO_TASA_CAMBIO_CURVATURA);

    // Actualizar variables globales para telemetría
    errorActual = (int16_t)errorFiltrado;
    curvaturaActual = curvatura;

    // Guardar último error significativo para retroceso inteligente
    if (abs(errorFiltrado) > 100) {
        ultimoErrorSignificativo = (int16_t)errorFiltrado;
    }

    // Actualizar historia
    errorAnterior = (int16_t)errorFiltrado;
    tiempoAnterior = tiempoActual;

    // 6. Ajustar PID según curvatura detectada (solo si el modo adaptativo está activo)
    if (pidAdaptativoActivo) {
        pid.ajustarParametros(curvatura);
    }

    // 7. Calcular velocidad adaptativa según curvatura (SIMPLIFICADO - 2 NIVELES)
    uint8_t velocidadActual = velocidadBase;

    if (curvatura >= UMBRAL_CURVA_CERRADA) {
        // Curva cerrada: reducir a 60% velocidad base
        velocidadActual = velocidadBase * FACTOR_VEL_CURVA_CERRADA;
    }
    // Si curvatura < UMBRAL_CURVA_CERRADA → usa velocidadBase (100%)

    // 8. Calcular corrección del PID usando error FILTRADO
    // Esto reduce oscilaciones causadas por ruido en sensores
    float correccion = pid.calcular(errorFiltrado);

    // 9. AMPLIFICACIÓN GRADUAL DE CORRECCIÓN EN ERRORES GRANDES
    // Calculamos un factor de amplificación que aumenta suavemente según el error
    float factorAmplificacion = 1.0;
    float errorAbs = abs(errorFiltrado);

    if (errorAbs > UMBRAL_AMPLIFICACION_MIN) {
        // Interpolación lineal entre umbral mínimo y máximo
        if (errorAbs >= UMBRAL_AMPLIFICACION_MAX) {
            factorAmplificacion = FACTOR_AMPLIFICACION_MAX;
        } else {
            // Transición suave: factor aumenta gradualmente
            float progreso = (errorAbs - UMBRAL_AMPLIFICACION_MIN) /
                           (UMBRAL_AMPLIFICACION_MAX - UMBRAL_AMPLIFICACION_MIN);
            factorAmplificacion = FACTOR_AMPLIFICACION_MIN +
                                (FACTOR_AMPLIFICACION_MAX - FACTOR_AMPLIFICACION_MIN) * progreso;
        }

        DEBUG_PRINT(DEBUG_PID, "⚡ Amplificación: ");
        DEBUG_PRINTLN(DEBUG_PID, factorAmplificacion);
    }

    // Aplicar amplificación a la corrección
    correccion *= factorAmplificacion;

    // Guardar la última corrección para usarla en caso de pérdida de línea
    ultimaCorreccionConocida = correccion;

    // 10. DETECCIÓN DE GIRO CRÍTICO (Curvas extremadamente cerradas)
    // Cuando el error supera el umbral crítico (solo sensores extremos activos),
    // activar modo PIVOTE: rueda interior a baja velocidad para girar sobre su eje
    if (errorAbs > UMBRAL_GIRO_CRITICO) {
        // Error crítico detectado: activar giro en pivote
        // Error NEGATIVO = línea a la IZQUIERDA (sensor izq con peso -4)
        bool giroIzquierda = (errorFiltrado < 0);  // Error negativo = línea a la izquierda

        motores.pivote(VELOCIDAD_PIVOTE_INTERIOR, VELOCIDAD_PIVOTE_EXTERIOR, giroIzquierda);

        DEBUG_PRINT(DEBUG_PID, "🔄 PIVOTE ACTIVADO | Error: ");
        DEBUG_PRINT(DEBUG_PID, errorFiltrado);
        DEBUG_PRINT(DEBUG_PID, " | Dirección: ");
        DEBUG_PRINTLN(DEBUG_PID, giroIzquierda ? "IZQ" : "DER");
    } else {
        // Control normal con PID + amplificación gradual

        // Limitar corrección para que ambas ruedas sigan avanzando
        float correccionMaxPermitida = min((float)CORRECCION_MAX, (float)(velocidadActual * CORRECCION_MAX_PORCENTAJE));

        if (correccion > correccionMaxPermitida) {
            correccion = correccionMaxPermitida;
        } else if (correccion < -correccionMaxPermitida) {
            correccion = -correccionMaxPermitida;
        }

        // 11. Aplicar corrección PID a los motores
        // INVERTIDO: Error negativo (sensor izq) debe girar a la IZQUIERDA
        int16_t velIzq = velocidadActual + correccion;
        int16_t velDer = velocidadActual - correccion;

        motores.diferencial(velIzq, velDer);
    }

    // Debug opcional
    DEBUG_PRINT(DEBUG_PID, "Curvatura: ");
    DEBUG_PRINT(DEBUG_PID, curvatura);
    DEBUG_PRINT(DEBUG_PID, " | Modo: ");
    DEBUG_PRINT(DEBUG_PID, pid.obtenerModoActual());
    DEBUG_PRINT(DEBUG_PID, " | Vel: ");
    DEBUG_PRINTLN(DEBUG_PID, velocidadActual);
}

/*******************************************************************************
 * ESTADO: PERDIDA_LINEA
 *
 * La línea se perdió temporalmente. Mantiene última dirección conocida
 * durante un tiempo antes de entrar en modo búsqueda.
 ******************************************************************************/
void estadoPerdidaLinea() {
    static bool mensajeRetrocesoMostrado = false;

    // Leer sensores
    sensores.leer();
    sensores.procesar();

    // Verificar si se recuperó la línea
    if (sensores.isLineaDetectada()) {
        Serial.println("✅ Línea recuperada!");
        mensajeRetrocesoMostrado = false;  // Reset para próxima pérdida
        cambiarEstado(SIGUIENDO_LINEA);
        return;
    }

    unsigned long tiempoTranscurrido = millis() - tiempoPerdidaLinea;

    // FASE 1: Tolerancia inicial (0-800ms)
    // Mantener última dirección con velocidad reducida
    if (tiempoTranscurrido <= TIMEOUT_PERDIDA_LINEA) {
        // En lugar de avanzar recto, aplicamos la última corrección conocida
        // para mantener la curva que probablemente causó la pérdida de línea.
        int16_t velIzq = VELOCIDAD_MIN + ultimaCorreccionConocida;
        int16_t velDer = VELOCIDAD_MIN - ultimaCorreccionConocida;
        motores.diferencial(velIzq, velDer);
        return;
    }

    // FASE 2: Retroceso inteligente (800-1500ms)
    // Retroceder girando hacia donde estaba la línea
    if (tiempoTranscurrido <= TIMEOUT_RETROCESO) {
        // Determinar dirección de giro según último error significativo
        // Error negativo = línea estaba a la izquierda
        // Error positivo = línea estaba a la derecha
        bool girarIzquierda = (ultimoErrorSignificativo < 0);

        // Usar velocidad base configurada por el usuario (en lugar de valor fijo)
        motores.retrocederConGiro(velocidadBase, FACTOR_GIRO_RETROCESO, girarIzquierda);

        // Mostrar mensaje solo en la primera ejecución de esta fase
        if (!mensajeRetrocesoMostrado) {
            Serial.print("🔄 Retroceso inteligente hacia ");
            Serial.println(girarIzquierda ? "IZQUIERDA" : "DERECHA");
            mensajeRetrocesoMostrado = true;
        }
        return;
    }

    // FASE 3: Búsqueda activa (después de 1500ms)
    // Si el retroceso no funcionó, pasar a búsqueda activa
    mensajeRetrocesoMostrado = false;  // Reset para próxima pérdida
    Serial.println("⚠️ Retroceso sin éxito. Iniciando búsqueda activa...");
    cambiarEstado(BUSCANDO_LINEA);
}

/*******************************************************************************
 * ESTADO: BUSCANDO_LINEA
 *
 * Búsqueda activa de la línea girando sobre su eje.
 * Alterna dirección de búsqueda.
 ******************************************************************************/
void estadoBuscarLinea() {
    // Leer sensores
    sensores.leer();
    sensores.procesar();

    // Verificar si se encontró la línea
    if (sensores.isLineaDetectada()) {
        Serial.println("¡Línea encontrada!");
        cambiarEstado(SIGUIENDO_LINEA);
        pid.reset();  // Reset PID para evitar transitorios
        return;
    }

    // Verificar timeout de búsqueda
    if (millis() - tiempoPerdidaLinea > TIMEOUT_BUSQUEDA) {
        Serial.println("Timeout de búsqueda. Deteniendo robot.");
        cambiarEstado(DETENIDO);
        return;
    }

    // Girar en dirección de búsqueda
    if (direccionBusqueda) {
        motores.girarDerecha(VELOCIDAD_BUSQUEDA);
    } else {
        motores.girarIzquierda(VELOCIDAD_BUSQUEDA);
    }

    // Alternar dirección cada cierto tiempo
    static unsigned long ultimoCambioDireccion = 0;
    if (millis() - ultimoCambioDireccion > 1000) {  // Cada 1 segundo
        direccionBusqueda = !direccionBusqueda;
        ultimoCambioDireccion = millis();
    }
}

/*******************************************************************************
 * ESTADO: DETENIDO
 *
 * Robot detenido. Espera comandos del usuario.
 ******************************************************************************/
void estadoDetenido() {
    motores.detener();

    // Mostrar mensaje solo una vez
    static bool mensajeMostrado = false;
    if (!mensajeMostrado) {
        Serial.println("\n========================================");
        Serial.println("ROBOT DETENIDO");
        Serial.println("========================================");
        Serial.println("Envíe 'r' para reiniciar");
        Serial.println("Envíe 'c' para recalibrar");
        Serial.println("========================================\n");
        mensajeMostrado = true;
    }

    // Reset flag cuando salga del estado
    if (estadoActual != DETENIDO) {
        mensajeMostrado = false;
    }
}

/*******************************************************************************
 * ESTADO: DIAGNOSTICO
 *
 * Modo de diagnóstico de hardware.
 ******************************************************************************/
void estadoDiagnostico() {
    Serial.println("\n========================================");
    Serial.println("MODO DIAGNÓSTICO");
    Serial.println("========================================\n");

    // Test de sensores
    Serial.println("1. PROBANDO SENSORES IR...");
    for (int i = 0; i < 5; i++) {
        sensores.leer();
        sensores.imprimirValores();
        delay(1000);
    }

    // Test de motores
    Serial.println("\n2. PROBANDO MOTORES...");
    motores.testMotores();

    // Volver a estado detenido
    Serial.println("\nDiagnóstico completado.");
    cambiarEstado(DETENIDO);
}

/*******************************************************************************
 * Cambia el estado del robot y notifica por serial
 ******************************************************************************/
void cambiarEstado(EstadoRobot nuevoEstado) {
    if (nuevoEstado != estadoActual) {
        estadoAnterior = estadoActual;
        estadoActual = nuevoEstado;

        if (DEBUG_ESTADOS) {
            Serial.print("\n>>> CAMBIO DE ESTADO: ");
            Serial.print(nombreEstado(estadoAnterior));
            Serial.print(" → ");
            Serial.println(nombreEstado(estadoActual));
        }
    }
}

/*******************************************************************************
 * Detiene los motores y cambia al estado especificado
 *
 * CONSOLIDACIÓN: Helper function para evitar código duplicado en múltiples
 * lugares donde se necesita detener los motores Y cambiar de estado.
 ******************************************************************************/
void detenerYCambiarEstado(EstadoRobot nuevoEstado) {
    motores.detener();
    cambiarEstado(nuevoEstado);
}

/*******************************************************************************
 * Retorna el nombre del estado como string
 ******************************************************************************/
const char* nombreEstado(EstadoRobot estado) {
    switch (estado) {
        case CALIBRANDO:      return "CALIBRANDO";
        case SIGUIENDO_LINEA: return "SIGUIENDO_LINEA";
        case PERDIDA_LINEA:   return "PERDIDA_LINEA";
        case BUSCANDO_LINEA:  return "BUSCANDO_LINEA";
        case PAUSADO:         return "PAUSADO";
        case CONFIGURACION:   return "CONFIGURACION";
        case DETENIDO:        return "DETENIDO";
        case DIAGNOSTICO:     return "DIAGNOSTICO";
        default:              return "DESCONOCIDO";
    }
}

/*******************************************************************************
 * Envía telemetría por Serial
 ******************************************************************************/
void enviarTelemetria() {
    unsigned long tiempoTranscurrido = (millis() - tiempoInicio) / 1000;

    Serial.println("\n========== TELEMETRÍA ==========");
    Serial.print("Estado: "); Serial.println(nombreEstado(estadoActual));
    Serial.print("Ciclos: "); Serial.print(ciclosProcesamiento);
    Serial.print(" | Tiempo: "); Serial.print(tiempoTranscurrido);
    Serial.println("s");
    Serial.println();

    // Control PID - Modo y Error actual
    Serial.print("Modo PID: ");
    Serial.print(pid.obtenerModoActual());
    Serial.print(" | Error actual: ");
    Serial.print(errorActual);
    Serial.print(" | Curvatura: ");
    Serial.println(curvaturaActual);

    // Valores de sensores
    sensores.imprimirValores();

    // Estado de motores
    Serial.print("Motores - Izq: ");
    Serial.print(motores.obtenerVelocidadIzquierdo());
    Serial.print(" | Der: ");
    Serial.println(motores.obtenerVelocidadDerecho());

    // Estadísticas PID
    Serial.print("Error promedio: ");
    Serial.println(pid.obtenerErrorPromedio());

    Serial.println("================================\n");
}

/*******************************************************************************
 * Procesa comandos recibidos por Serial
 *
 * OPTIMIZACIÓN: Usa buffer de tamaño fijo en vez de String dinámico para:
 *   ✓ Evitar fragmentación de heap (problema común en ESP32 con Strings)
 *   ✓ Menor uso de memoria (64 bytes fijos vs heap dinámico)
 *   ✓ Mejor rendimiento (sin malloc/free)
 *   ✓ Comportamiento determinístico
 *
 * Comandos disponibles:
 *   c          - Iniciar calibración
 *   s          - Mostrar estado actual
 *   r          - Reset/reiniciar
 *   d          - Modo diagnóstico
 *   p [Kp Ki Kd] - Ajustar PID
 *   v [vel]    - Cambiar velocidad base
 *   h          - Mostrar ayuda
 ******************************************************************************/
void procesarComandosSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (comandoIndex > 0) {
                comandoBuffer[comandoIndex] = '\0';  // Null-terminator
                String comandoSerial = String(comandoBuffer);  // Conversión temporal para compatibilidad
                ejecutarComando(comandoSerial);
                comandoIndex = 0;  // Reset del buffer
            }
        } else if (comandoIndex < COMANDO_BUFFER_SIZE - 1) {
            comandoBuffer[comandoIndex++] = c;
        } else {
            // Buffer lleno - descartar comando y advertir
            Serial.println("✗ Error: Comando demasiado largo (max 64 caracteres)");
            comandoIndex = 0;
        }
    }
}

/*******************************************************************************
 * Ejecuta un comando recibido por Serial
 ******************************************************************************/
void ejecutarComando(String cmd) {
    cmd.trim();
    cmd.toLowerCase();

    // Ignorar comandos vacíos
    if (cmd.length() == 0) return;

    Serial.print("\n> ");
    Serial.println(cmd);

    // ========== COMANDOS DE CONTROL ==========

    // Comando: 0 - Pausar robot (atajo rápido)
    if (cmd == "0") {
        if (estadoActual == SIGUIENDO_LINEA ||
            estadoActual == PERDIDA_LINEA ||
            estadoActual == BUSCANDO_LINEA) {
            Serial.println("✓ Pausando robot...");
            robotPausado = true;
            cambiarEstado(PAUSADO);
        } else {
            Serial.println("✗ No se puede pausar en el estado actual");
            Serial.print("  Estado: "); Serial.println(nombreEstado(estadoActual));
        }
    }

    // Comando: 1 - Reanudar robot (atajo rápido)
    else if (cmd == "1") {
        if (estadoActual == PAUSADO || estadoActual == CONFIGURACION) {
            Serial.println("✓ Reanudando operación...");
            robotPausado = false;
            cambiarEstado(SIGUIENDO_LINEA);
        } else {
            Serial.println("✗ Robot no está pausado");
        }
    }

    // Comando: pause / p - Pausar robot
    else if (cmd == "pause" || cmd == "pausa") {
        if (estadoActual == SIGUIENDO_LINEA ||
            estadoActual == PERDIDA_LINEA ||
            estadoActual == BUSCANDO_LINEA) {
            Serial.println("✓ Pausando robot...");
            robotPausado = true;
            cambiarEstado(PAUSADO);
        } else {
            Serial.println("✗ No se puede pausar en el estado actual");
            Serial.print("  Estado: "); Serial.println(nombreEstado(estadoActual));
        }
    }

    // Comando: resume / continuar - Reanudar robot
    else if (cmd == "resume" || cmd == "continuar" || cmd == "reanudar") {
        if (estadoActual == PAUSADO || estadoActual == CONFIGURACION) {
            Serial.println("✓ Reanudando operación...");
            robotPausado = false;
            cambiarEstado(SIGUIENDO_LINEA);
        } else {
            Serial.println("✗ Robot no está pausado");
        }
    }

    // Comando: stop - Detener completamente
    else if (cmd == "stop" || cmd == "detener") {
        Serial.println("✓ Deteniendo robot...");
        detenerYCambiarEstado(DETENIDO);
    }

    // ========== COMANDOS DE CONFIGURACIÓN ==========

    // Comando: config / cfg - Modo configuración
    else if (cmd == "config" || cmd == "cfg") {
        if (estadoActual != CALIBRANDO) {
            Serial.println("✓ Entrando en modo configuración...");
            cambiarEstado(CONFIGURACION);
        } else {
            Serial.println("✗ No disponible durante calibración");
        }
    }

    // Comando: p [modo] [Kp] [Ki] [Kd] - Ajustar PID (múltiples sintaxis)
    else if (cmd.startsWith("p ") || cmd.startsWith("pid ")) {
        String args = cmd.substring(cmd.indexOf(' ') + 1);
        args.trim();

        // Detectar si el primer argumento es un modo (recta/cerrada)
        bool esModoEspecifico = false;
        PIDParams* modoTarget = nullptr;
        String nombreModo = "";

        if (args.startsWith("recta ")) {
            esModoEspecifico = true;
            modoTarget = &PID_RECTA;
            nombreModo = "RECTA";
            args = args.substring(6); // Remover "recta "
        } else if (args.startsWith("cerrada ")) {
            esModoEspecifico = true;
            modoTarget = &PID_CURVA_CERRADA;
            nombreModo = "CURVA_CERRADA";
            args = args.substring(8); // Remover "cerrada "
        }

        // Parsear argumentos
        float kp, ki, kd;
        int n = sscanf(args.c_str(), "%f %f %f", &kp, &ki, &kd);

        if (n >= 1 && n <= 3) {
            // Obtener valores actuales
            float kp_actual, ki_actual, kd_actual;
            if (esModoEspecifico) {
                kp_actual = modoTarget->Kp;
                ki_actual = modoTarget->Ki;
                kd_actual = modoTarget->Kd;
            } else {
                pid.getParametros(kp_actual, ki_actual, kd_actual);
            }

            // Aplicar solo los valores especificados
            if (n >= 1) kp_actual = kp;
            if (n >= 2) ki_actual = ki;
            if (n >= 3) kd_actual = kd;

            // Validar rangos
            if (kp_actual >= 0 && kp_actual <= 10 && ki_actual >= 0 && ki_actual <= 5 && kd_actual >= 0 && kd_actual <= 10) {
                if (esModoEspecifico) {
                    // Modificar modo específico
                    modoTarget->Kp = kp_actual;
                    modoTarget->Ki = ki_actual;
                    modoTarget->Kd = kd_actual;
                    Serial.print("✓ Modo "); Serial.print(nombreModo); Serial.println(" actualizado:");
                    Serial.print("  Kp="); Serial.print(kp_actual, 3);
                    Serial.print(" | Ki="); Serial.print(ki_actual, 3);
                    Serial.print(" | Kd="); Serial.println(kd_actual, 3);
                    // Si el adaptativo está activo, se aplicará automáticamente
                } else {
                    // Modo MANUAL: establecer valores fijos y desactivar adaptativo
                    pid.setParametros(kp_actual, ki_actual, kd_actual);
                    pidAdaptativoActivo = false;
                    Serial.println("✓ PID en MODO MANUAL (fijo):");
                    Serial.print("  Kp="); Serial.print(kp_actual, 3);
                    Serial.print(" | Ki="); Serial.print(ki_actual, 3);
                    Serial.print(" | Kd="); Serial.println(kd_actual, 3);
                    Serial.println("  💡 Usa 'pa' para reactivar modo adaptativo");
                }
                Serial.println("💾 Tip: Use 'save' para guardar en Flash");
                flagConfigChanged = true;
                guardarConfigPendiente = true;
            } else {
                Serial.println("✗ Valores fuera de rango");
                Serial.println("  Kp: 0-10, Ki: 0-5, Kd: 0-10");
            }
        } else {
            Serial.println("✗ Formato inválido");
            Serial.println("\n📝 Sintaxis disponibles:");
            Serial.println("  p <Kp> <Ki> <Kd>           → Modo MANUAL");
            Serial.println("  p <Kp> <Ki>                → Mantiene Kd actual");
            Serial.println("  p <Kp>                     → Mantiene Ki y Kd actuales");
            Serial.println("  p recta <Kp> <Ki> <Kd>    → Modifica modo RECTA");
            Serial.println("  p cerrada <Kp> <Ki> <Kd>  → Modifica modo CURVA_CERRADA");
            Serial.println("\n💡 Ejemplo: p 2.0 0.1 1.5  o  p recta 1.0 0.005 0.5");
        }
    }

    // Comando: pa - Activar PID adaptativo
    else if (cmd == "pa" || cmd == "adaptativo") {
        pidAdaptativoActivo = true;
        Serial.println("✓ Modo PID ADAPTATIVO activado (2 modos)");
        Serial.println("  Los parámetros cambiarán según curvatura:");
        Serial.print("  - RECTA:         Kp="); Serial.print(PID_RECTA.Kp, 3);
        Serial.print(" Ki="); Serial.print(PID_RECTA.Ki, 3);
        Serial.print(" Kd="); Serial.println(PID_RECTA.Kd, 3);
        Serial.print("  - CURVA_CERRADA: Kp="); Serial.print(PID_CURVA_CERRADA.Kp, 3);
        Serial.print(" Ki="); Serial.print(PID_CURVA_CERRADA.Ki, 3);
        Serial.print(" Kd="); Serial.println(PID_CURVA_CERRADA.Kd, 3);
    }

    // Comando: v [vel] / vel [vel] - Cambiar velocidad base
    else if (cmd.startsWith("v ") || cmd.startsWith("vel ")) {
        int vel;
        int offset = cmd.startsWith("vel ") ? 4 : 2;

        if (sscanf(cmd.c_str() + offset, "%d", &vel) == 1) {
            if (vel >= VELOCIDAD_MIN && vel <= VELOCIDAD_MAX) {
                velocidadBase = vel;
                Serial.println("✓ Velocidad base ajustada:");
                Serial.print("  Nueva velocidad: "); Serial.println(velocidadBase);
                Serial.print("  Rango: "); Serial.print(VELOCIDAD_MIN);
                Serial.print("-"); Serial.println(VELOCIDAD_MAX);
                Serial.println("💾 Tip: Use 'save' para guardar en Flash");
                flagConfigChanged = true;
                guardarConfigPendiente = true;
            } else {
                Serial.println("✗ Velocidad fuera de rango");
                Serial.print("  Permitido: "); Serial.print(VELOCIDAD_MIN);
                Serial.print("-"); Serial.println(VELOCIDAD_MAX);
            }
        } else {
            Serial.println("✗ Formato inválido");
            Serial.println("  Uso: v <velocidad>");
            Serial.println("  Ejemplo: v 180");
        }
    }

    // ========== COMANDOS DE SISTEMA ==========

    // Comando: c / calibrar - Iniciar calibración
    else if (cmd == "c" || cmd == "calibrar") {
        Serial.println("✓ Iniciando calibración...");
        Serial.println("  Mueva el robot sobre blanco y negro durante 5s");
        sensores.iniciarCalibracion();
        cambiarEstado(CALIBRANDO);
    }

    // Comando: s / status / estado - Mostrar estado
    else if (cmd == "s" || cmd == "status" || cmd == "estado") {
        Serial.println("\n╔════════════════════════════════════════╗");
        Serial.println("║       ESTADO DEL SISTEMA               ║");
        Serial.println("╚════════════════════════════════════════╝");
        Serial.print("Estado: "); Serial.println(nombreEstado(estadoActual));
        Serial.print("Ciclos: "); Serial.println(ciclosProcesamiento);
        Serial.print("Tiempo: "); Serial.print((millis() - tiempoInicio) / 1000);
        Serial.println(" s");
        Serial.print("Velocidad base: "); Serial.println(velocidadBase);
        Serial.println();

        // Mostrar PID actual en memoria
        float kp, ki, kd;
        pid.getParametros(kp, ki, kd);
        Serial.println("📊 PID ACTUAL EN MEMORIA:");
        Serial.print("  Kp="); Serial.print(kp, 3);
        Serial.print(" | Ki="); Serial.print(ki, 3);
        Serial.print(" | Kd="); Serial.println(kd, 3);
        Serial.print("  Modo: ");
        Serial.println(pidAdaptativoActivo ? "ADAPTATIVO ✓" : "MANUAL (fijo)");
        Serial.println();

        // Mostrar parámetros configurados para cada modo
        Serial.println("⚙️  PARÁMETROS PID CONFIGURADOS:");
        Serial.println("┌──────────────┬────────┬────────┬────────┐");
        Serial.println("│ Modo         │   Kp   │   Ki   │   Kd   │");
        Serial.println("├──────────────┼────────┼────────┼────────┤");

        Serial.print("│ RECTA          │ ");
        Serial.print(PID_RECTA.Kp, 3); Serial.print(" │ ");
        Serial.print(PID_RECTA.Ki, 3); Serial.print(" │ ");
        Serial.print(PID_RECTA.Kd, 3); Serial.println(" │");

        Serial.print("│ CURVA_CERRADA  │ ");
        Serial.print(PID_CURVA_CERRADA.Kp, 3); Serial.print(" │ ");
        Serial.print(PID_CURVA_CERRADA.Ki, 3); Serial.print(" │ ");
        Serial.print(PID_CURVA_CERRADA.Kd, 3); Serial.println(" │");

        Serial.println("└────────────────┴────────┴────────┴────────┘");
        Serial.println();

        // Mostrar umbral de curvatura
        Serial.println("🔄 UMBRAL DE CURVATURA:");
        Serial.print("  Curva cerrada: > "); Serial.println(UMBRAL_CURVA_CERRADA);
        Serial.println();
        Serial.println("⚡ AMPLIFICACIÓN DE CORRECCIÓN:");
        Serial.print("  Umbral inicio: > "); Serial.println(UMBRAL_AMPLIFICACION_MIN);
        Serial.print("  Umbral máximo: > "); Serial.println(UMBRAL_AMPLIFICACION_MAX);
        Serial.print("  Factor mínimo: "); Serial.print(FACTOR_AMPLIFICACION_MIN); Serial.println("x");
        Serial.print("  Factor máximo: "); Serial.print(FACTOR_AMPLIFICACION_MAX); Serial.println("x");
        Serial.println("========================================");

        // Opciones según estado
        if (estadoActual == PAUSADO) {
            Serial.println("\n💡 Opciones disponibles:");
            Serial.println("  'resume' / '1'       → Continuar");
            Serial.println("  'p <Kp> <Ki> <Kd>'   → PID manual");
            Serial.println("  'p recta <...>'      → Ajustar modo RECTA");
            Serial.println("  'p cerrada <...>'    → Ajustar modo CURVA_CERRADA");
            Serial.println("  'pa'                 → Activar PID adaptativo");
            Serial.println("  'v <vel>'            → Cambiar velocidad");
        }
        Serial.println();
    }

    // Comando: r / reset / reiniciar - Reset
    else if (cmd == "r" || cmd == "reset" || cmd == "reiniciar") {
        Serial.println("✓ Reiniciando sistema...");
        pid.reset();
        motores.detener();
        delay(500);
        cambiarEstado(SIGUIENDO_LINEA);
        Serial.println("  Sistema reiniciado");
    }

    // Comando: d / diag / diagnostico - Diagnóstico
    else if (cmd == "d" || cmd == "diag" || cmd == "diagnostico") {
        Serial.println("✓ Entrando en modo diagnóstico...");
        detenerYCambiarEstado(DIAGNOSTICO);
    }

    // Comando: h / help / ayuda - Ayuda
    else if (cmd == "h" || cmd == "help" || cmd == "ayuda" || cmd == "?") {
        mostrarAyuda();
    }

    // ========== COMANDOS DE PERSISTENCIA NVS ==========

    // Comando: save / guardar - Guardar configuración en Flash
    else if (cmd == "save" || cmd == "guardar") {
        float kp, ki, kd;
        pid.getParametros(kp, ki, kd);

        if (configNVS.guardarConfiguracion(velocidadBase, kp, ki, kd)) {
            Serial.println("💾 Los valores actuales se mantendrán después de apagar el ESP32");
            guardarConfigPendiente = false;
        } else {
            Serial.println("✗ Error al guardar configuración");
        }
    }

    // Comando: load / cargar - Recargar configuración desde Flash
    else if (cmd == "load" || cmd == "cargar") {
        float kp, ki, kd;
        uint8_t vel;

        if (configNVS.cargarConfiguracion(vel, kp, ki, kd)) {
            velocidadBase = vel;
            pid.setParametros(kp, ki, kd);
            Serial.println("✓ Configuración recargada desde Flash");
        } else {
            Serial.println("✗ No hay configuración guardada");
        }
    }

    // Comando: reset_config - Restaurar valores por defecto
    else if (cmd == "reset_config" || cmd == "restaurar") {
        if (configNVS.restaurarDefecto()) {
            // Aplicar valores por defecto de config.h
            velocidadBase = VELOCIDAD_BASE;
            pid.setParametros(PID_RECTA.Kp, PID_RECTA.Ki, PID_RECTA.Kd);
            Serial.println("✓ Valores por defecto aplicados:");
            Serial.print("  Velocidad: "); Serial.println(VELOCIDAD_BASE);
            Serial.print("  PID: Kp="); Serial.print(PID_RECTA.Kp);
            Serial.print(" Ki="); Serial.print(PID_RECTA.Ki);
            Serial.print(" Kd="); Serial.println(PID_RECTA.Kd);
        }
    }

    // Comando: nvs_info - Mostrar información de NVS
    else if (cmd == "nvs_info" || cmd == "info_nvs") {
        configNVS.mostrarInfoNVS();
    }

    // ========== COMANDOS AVANZADOS ==========

    // Comando: info - Información detallada
    else if (cmd == "info") {
        Serial.println("\n╔════════════════════════════════════════╗");
        Serial.println("║       INFORMACIÓN DETALLADA            ║");
        Serial.println("╚════════════════════════════════════════╝");
        sensores.imprimirValores();
        motores.imprimirEstado();
        pid.imprimirEstado();
        Serial.println("========================================\n");
    }

    // Comando: test - Test de motores
    else if (cmd == "test") {
        if (estadoActual == DETENIDO || estadoActual == PAUSADO) {
            Serial.println("✓ Iniciando test de motores...");
            motores.testMotores();
        } else {
            Serial.println("✗ Pause el robot primero (comando: pause)");
        }
    }

    // ========== COMANDOS DE TEST ==========

    // Comando: w - Test ambos motores a velocidad del programa
    else if (cmd == "w") {
        if (estadoActual == DETENIDO || estadoActual == PAUSADO) {
            Serial.println("✓ Test: Ambos motores adelante a VELOCIDAD_BASE");
            Serial.print("  Velocidad configurada: ");
            Serial.println(VELOCIDAD_BASE);
            Serial.println("  Presione '0' para detener");
            motores.diferencial(VELOCIDAD_BASE, VELOCIDAD_BASE);
        } else {
            Serial.println("✗ Pause el robot primero (comando: 0)");
        }
    }

    // Comando: ts - Test de sensores en tiempo real
    else if (cmd == "ts") {
        Serial.println("✓ Iniciando test de sensores...");
        Serial.println("  Mostrando valores cada 500ms. Presione 'x' para detener.\n");

        while (!Serial.available() || Serial.read() != 'x') {
            sensores.leer();
            int16_t error = sensores.procesar();
            bool lineaDetectada = sensores.isLineaDetectada();

            Serial.print("Error: ");
            Serial.print(error);
            Serial.print(" | Línea: ");
            Serial.println(lineaDetectada ? "SÍ" : "NO");
            sensores.imprimirValores(); // Esta función ahora imprime los detalles

            delay(500);
        }
        while(Serial.available()) Serial.read(); // Limpiar buffer de entrada
        Serial.println("\n✓ Test de sensores finalizado");
    }

    // Comando: tm - Test completo de motores
    else if (cmd == "tm") {
        if (estadoActual == DETENIDO || estadoActual == PAUSADO) {
            Serial.println("✓ Test completo de motores");
            Serial.println("  1. Ambos adelante 2 seg");
            Serial.println("  2. Ambos atrás 2 seg");
            Serial.println("  3. Giro izquierda 1 seg");
            Serial.println("  4. Giro derecha 1 seg\n");

            // Test adelante
            Serial.println("→ Adelante...");
            motores.diferencial(150, 150);
            delay(2000);

            // Pausa
            motores.detener();
            delay(500);

            // Test atrás
            Serial.println("← Atrás...");
            motores.diferencial(-150, -150);
            delay(2000);

            // Pausa
            motores.detener();
            delay(500);

            // Test giro izquierda
            Serial.println("↺ Giro izquierda...");
            motores.diferencial(-100, 100);
            delay(1000);

            // Pausa
            motores.detener();
            delay(500);

            // Test giro derecha
            Serial.println("↻ Giro derecha...");
            motores.diferencial(100, -100);
            delay(1000);

            // Detener
            motores.detener();
            Serial.println("\n✓ Test completo finalizado");
        } else {
            Serial.println("✗ Pause el robot primero (comando: 0)");
        }
    }

    // Comando: tp - Test PID en tiempo real
    else if (cmd == "tp") {
        Serial.println("✓ Monitor PID en tiempo real");
        Serial.println("  Mostrando cálculos PID cada 100ms");
        Serial.println("  Presione 'x' para detener\n");
        Serial.println("Error | P_term | I_term | D_term | Output");
        Serial.println("──────┼────────┼────────┼────────┼────────");

        for (int i = 0; i < 30; i++) {
            sensores.leer();
            int16_t error = sensores.procesar();
            int16_t correccion = pid.calcular(error);

            Serial.print(error);
            Serial.print("\t| ");
            Serial.print(pid.obtenerP());
            Serial.print("\t| ");
            Serial.print(pid.obtenerI());
            Serial.print("\t| ");
            Serial.print(pid.obtenerD());
            Serial.print("\t| ");
            Serial.println(correccion);

            delay(100);

            if (Serial.available() > 0) {
                char c = Serial.read();
                if (c == 'x' || c == 'X') {
                    Serial.println("\n✓ Monitor detenido");
                    break;
                }
            }
        }
        Serial.println("\n✓ Monitor PID finalizado");
    }

    // Comando: tc - Test de detección de curvatura (nuevo)
    else if (cmd == "tc") {
        Serial.println("✓ Monitor de detección de curvatura");
        Serial.println("  Mostrando análisis de curvatura en tiempo real");
        Serial.println("  Presione 'x' para detener\n");
        Serial.println("Error | Filtrado | TasaCambio | Curvatura | Modo PID | Vel% | Giro");
        Serial.println("──────┼──────────┼────────────┼───────────┼──────────┼──────┼──────");

        int16_t errorPrevio = 0;
        float errorFiltradoTest = 0;
        unsigned long tiempoPrevio = millis();

        while (true) {
            sensores.leer();
            int16_t error = sensores.procesar();

            // Aplicar filtro (misma lógica que estadoSeguirLinea)
            const float ALPHA_FILTRO = 0.7;
            errorFiltradoTest = ALPHA_FILTRO * error + (1.0 - ALPHA_FILTRO) * errorFiltradoTest;

            // Banda muerta
            float errorFiltradoFinal = errorFiltradoTest;
            if (abs(errorFiltradoFinal) < ERROR_DEADBAND) {
                errorFiltradoFinal = 0;
            }

            // Calcular curvatura (misma lógica que estadoSeguirLinea)
            unsigned long tiempoActual = millis();
            float dt = (tiempoActual - tiempoPrevio) / 1000.0;
            if (dt <= 0 || dt > 1.0) dt = 0.01;

            float tasaCambio = abs(errorFiltradoFinal - errorPrevio) / dt;
            uint16_t curvatura = (uint16_t)(abs(errorFiltradoFinal) * PESO_ERROR_CURVATURA +
                                             tasaCambio * PESO_TASA_CAMBIO_CURVATURA);

            // Determinar modo PID (SIMPLIFICADO - 2 MODOS)
            const char* modoPID;
            uint8_t velPorcentaje;
            if (curvatura >= UMBRAL_CURVA_CERRADA) {
                modoPID = "CERRADA";
                velPorcentaje = FACTOR_VEL_CURVA_CERRADA * 100;
            } else {
                modoPID = "RECTA";
                velPorcentaje = 100;
            }

            // Calcular factor de amplificación
            float factorAmp = 1.0;
            float errorAbsTest = abs(errorFiltradoFinal);
            if (errorAbsTest > UMBRAL_AMPLIFICACION_MIN) {
                if (errorAbsTest >= UMBRAL_AMPLIFICACION_MAX) {
                    factorAmp = FACTOR_AMPLIFICACION_MAX;
                } else {
                    float progreso = (errorAbsTest - UMBRAL_AMPLIFICACION_MIN) /
                                   (UMBRAL_AMPLIFICACION_MAX - UMBRAL_AMPLIFICACION_MIN);
                    factorAmp = FACTOR_AMPLIFICACION_MIN +
                              (FACTOR_AMPLIFICACION_MAX - FACTOR_AMPLIFICACION_MIN) * progreso;
                }
            }

            // Imprimir datos (ahora incluye error filtrado y amplificación)
            Serial.print(error);
            Serial.print("\t| ");
            Serial.print(errorFiltradoFinal, 1);
            Serial.print("\t| ");
            Serial.print(tasaCambio, 1);
            Serial.print("\t| ");
            Serial.print(curvatura);
            Serial.print("\t| ");
            Serial.print(modoPID);
            Serial.print("\t| ");
            Serial.print(velPorcentaje);
            Serial.print("%\t| ");
            if (factorAmp > 1.0) {
                Serial.print("⚡");
                Serial.print(factorAmp, 1);
                Serial.println("x");
            } else {
                Serial.println("1.0x");
            }

            errorPrevio = (int16_t)errorFiltradoFinal;
            tiempoPrevio = tiempoActual;

            delay(100);

            // Verificar si se presionó 'x'
            if (Serial.available() > 0) {
                char c = Serial.read();
                if (c == 'x' || c == 'X') {
                    while(Serial.available()) Serial.read(); // Limpiar buffer
                    Serial.println("\n✓ Monitor de curvatura detenido");
                    break;
                }
            }
        }
    }

    // Comando desconocido
    else {
        Serial.println("✗ Comando desconocido");
        Serial.println("  Escriba 'h' o '?' para ver ayuda");
    }
}

/*******************************************************************************
 * Muestra la ayuda de comandos
 ******************************************************************************/
void mostrarAyuda() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║       COMANDOS DISPONIBLES             ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    Serial.println("📋 CONTROL DEL ROBOT:");
    Serial.println("  0                  - Pausar robot (atajo rápido)");
    Serial.println("  1                  - Reanudar operación (atajo rápido)");
    Serial.println("  pause / pausa      - Pausar robot (detiene motores)");
    Serial.println("  resume / continuar - Reanudar operación");
    Serial.println("  stop / detener     - Detener completamente");
    Serial.println();
    Serial.println("⚙️  CONFIGURACIÓN PID:");
    Serial.println("  p <Kp> <Ki> <Kd>           - PID manual (fijo)");
    Serial.println("  p <Kp> <Ki>                - Modifica Kp y Ki (mantiene Kd)");
    Serial.println("  p <Kp>                     - Modifica solo Kp");
    Serial.println("  p recta <Kp> <Ki> <Kd>    - Ajusta modo RECTA");
    Serial.println("  p cerrada <Kp> <Ki> <Kd>  - Ajusta modo CURVA_CERRADA");
    Serial.println("  pa / adaptativo            - Activa PID adaptativo (2 modos)");
    Serial.println();
    Serial.println("⚙️  OTROS AJUSTES:");
    Serial.println("  v <velocidad>      - Cambiar velocidad base (0-255)");
    Serial.println("                       Ej: v 180");
    Serial.println("  config / cfg       - Modo configuración interactiva");
    Serial.println();
    Serial.println("🔧 SISTEMA:");
    Serial.println("  c / calibrar       - Iniciar calibración de sensores");
    Serial.println("  s / status         - Mostrar estado del sistema");
    Serial.println("  r / reset          - Reiniciar sistema");
    Serial.println("  d / diagnostico    - Modo diagnóstico de hardware");
    Serial.println();
    Serial.println("💾 PERSISTENCIA (NVS):");
    Serial.println("  save / guardar     - Guardar config actual en Flash");
    Serial.println("  load / cargar      - Recargar config desde Flash");
    Serial.println("  reset_config       - Restaurar valores por defecto");
    Serial.println("  nvs_info           - Info de almacenamiento NVS");
    Serial.println();
    Serial.println("📊 INFORMACIÓN:");
    Serial.println("  info               - Información detallada completa");
    Serial.println("  h / ? / ayuda      - Mostrar esta ayuda");
    Serial.println();
    Serial.println("🧪 COMANDOS DE TEST:");
    Serial.println("  w                  - Test motores a VELOCIDAD_BASE (adelante)");
    Serial.println("  ts                 - Test sensores en tiempo real (20 lecturas)");
    Serial.println("  tm                 - Test completo motores (secuencia 4 pasos)");
    Serial.println("  tp                 - Monitor PID en tiempo real (30 ciclos)");
    Serial.println("  tc                 - Monitor detección de curvatura (PID adaptativo)");
    Serial.println("  test               - Test básico de motores");
    Serial.println();
    Serial.println("  💡 Nota: Use comando '0' para detener tests de motores");
    Serial.println("           Use 'x' para salir de tests de sensores/PID/curvatura");
    Serial.println();
    Serial.println("🔘 BOTONES FÍSICOS:");
    Serial.println("  GPIO0  (BOOT)      - Pausar/Reanudar");
    Serial.println("  GPIO47             - Modo configuración");
    Serial.println("  GPIO48 (LED)       - Parada de emergencia");
    Serial.println();
    Serial.println("💡 EJEMPLOS DE USO:");
    Serial.println("  0                  → Atajo rápido para pausar");
    Serial.println("  1                  → Atajo rápido para reanudar");
    Serial.println("  p 1.5 0.05 0.8     → Ajusta PID para recta");
    Serial.println("  v 150              → Reduce velocidad a 150");
    Serial.println("  save               → Guarda config en Flash (persiste)");
    Serial.println("  s                  → Muestra estado y configuración");
    Serial.println();
    Serial.println("ℹ️  NOTA: Use 'save' después de ajustar parámetros para");
    Serial.println("   que se mantengan después de apagar el ESP32");
    Serial.println("========================================\n");
}

/*******************************************************************************
 * Inicializa botones e interrupciones
 ******************************************************************************/
void inicializarBotones() {
    Serial.println("Inicializando botones e interrupciones...");

    // Configurar pines como entrada con pull-up interno
    pinMode(BTN_PAUSE_RESUME, INPUT_PULLUP);
    pinMode(BTN_MODE_CHANGE, INPUT_PULLUP);
    pinMode(BTN_EMERGENCY_STOP, INPUT_PULLUP);

    // Adjuntar interrupciones (FALLING porque usamos pull-up)
    attachInterrupt(digitalPinToInterrupt(BTN_PAUSE_RESUME), isrPauseResume, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_MODE_CHANGE), isrModeChange, FALLING);
    attachInterrupt(digitalPinToInterrupt(BTN_EMERGENCY_STOP), isrEmergencyStop, FALLING);

    Serial.println("  Botones configurados:");
    Serial.print("    GPIO"); Serial.print(BTN_PAUSE_RESUME);
    Serial.println(" - Pausar/Reanudar");
    Serial.print("    GPIO"); Serial.print(BTN_MODE_CHANGE);
    Serial.println(" - Cambiar Modo");
    Serial.print("    GPIO"); Serial.print(BTN_EMERGENCY_STOP);
    Serial.println(" - Parada Emergencia");
    Serial.println("Botones inicializados correctamente\n");
}

/*******************************************************************************
 * Procesa las banderas de interrupciones
 ******************************************************************************/
void procesarBanderas() {
    // Parada de emergencia tiene máxima prioridad
    if (flagEmergencyStop) {
        flagEmergencyStop = false;
        Serial.println("\n¡¡¡ PARADA DE EMERGENCIA ACTIVADA !!!");
        detenerYCambiarEstado(DETENIDO);
        return;
    }

    // Pausa/Reanudación
    if (flagPauseResume) {
        flagPauseResume = false;

        if (estadoActual == PAUSADO) {
            // Reanudar
            Serial.println("\nReanudando operación...");
            robotPausado = false;
            cambiarEstado(SIGUIENDO_LINEA);
        } else if (estadoActual == SIGUIENDO_LINEA ||
                   estadoActual == PERDIDA_LINEA ||
                   estadoActual == BUSCANDO_LINEA) {
            // Pausar
            Serial.println("\nPausando robot...");
            robotPausado = true;
            cambiarEstado(PAUSADO);
        }
    }

    // Cambio de modo
    if (flagModeChange) {
        flagModeChange = false;

        if (estadoActual != CONFIGURACION && estadoActual != CALIBRANDO) {
            Serial.println("\nEntrando en modo configuración...");
            cambiarEstado(CONFIGURACION);
        }
    }
}

/*******************************************************************************
 * ESTADO: PAUSADO
 *
 * Robot pausado. Motores detenidos, esperando ajustes o reanudación.
 ******************************************************************************/
void estadoPausado() {
    // Detener motores
    motores.detener();

    // Mostrar mensaje solo una vez
    static bool mensajeMostrado = false;
    if (!mensajeMostrado) {
        Serial.println("\n========================================");
        Serial.println("ROBOT PAUSADO");
        Serial.println("========================================");
        Serial.println("Motores detenidos.");
        Serial.println("Puede ajustar parámetros:");
        Serial.println("  - 'p [Kp Ki Kd]' para ajustar PID");
        Serial.println("  - 'v [vel]' para cambiar velocidad");
        Serial.println("  - 'config' para menú de configuración");
        Serial.println("  - Presione BOOT o 'resume' para continuar");
        Serial.println("========================================\n");
        mensajeMostrado = true;
        permitirAjustes = true;
    }

    // Reset flag cuando salga del estado
    if (estadoActual != PAUSADO) {
        mensajeMostrado = false;
        permitirAjustes = false;
    }
}

/*******************************************************************************
 * ESTADO: CONFIGURACION
 *
 * Modo de configuración interactiva.
 ******************************************************************************/
void estadoConfiguracion() {
    // Detener motores
    motores.detener();

    // Mostrar menú solo una vez
    static bool menuMostrado = false;
    if (!menuMostrado) {
        mostrarMenuConfiguracion();
        menuMostrado = true;
    }

    // Reset flag cuando salga del estado
    if (estadoActual != CONFIGURACION) {
        menuMostrado = false;
    }
}

/*******************************************************************************
 * Muestra el menú de configuración interactiva
 ******************************************************************************/
void mostrarMenuConfiguracion() {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║    MODO CONFIGURACIÓN INTERACTIVA      ║");
    Serial.println("╚════════════════════════════════════════╝");
    Serial.println();
    Serial.println("AJUSTES DISPONIBLES:");
    Serial.println("  1. Parámetros PID");
    Serial.println("  2. Velocidades");
    Serial.println("  3. Sensores");
    Serial.println("  4. Ver configuración actual");
    Serial.println("  5. Salir (iniciar robot)");
    Serial.println();
    Serial.print("Configuración actual - Vel: ");
    Serial.print(velocidadBase);
    Serial.print(" | PID: Kp=");
    float kp, ki, kd;
    pid.getParametros(kp, ki, kd);
    Serial.print(kp);
    Serial.print(" Ki=");
    Serial.print(ki);
    Serial.print(" Kd=");
    Serial.println(kd);
    Serial.println();
    Serial.println("Envíe comando o 'r' para salir");
    Serial.println("========================================");
}
