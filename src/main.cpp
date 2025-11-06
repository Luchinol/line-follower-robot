/*******************************************************************************
 * MAIN.CPP - Carrito Seguidor de Línea ESP32
 *
 * Sistema de seguimiento de línea autónomo con:
 * - Doble array de sensores IR (anticipación + precisión)
 * - Control PID adaptativo según curvatura
 * - Máquina de estados para gestión de comportamiento
 * - Recuperación automática ante pérdida de línea
 * - Telemetría en tiempo real
 *
 * Estados del robot:
 *   CALIBRANDO      → Calibración automática de sensores
 *   SIGUIENDO_LINEA → Seguimiento normal de la línea
 *   PERDIDA_LINEA   → Línea perdida temporalmente (< 500ms)
 *   BUSCANDO_LINEA  → Búsqueda activa de la línea (girando)
 *   DETENIDO        → Robot detenido (error o fin de pista)
 *   DIAGNOSTICO     → Modo de diagnóstico de hardware
 *
 * Hardware:
 *   - ESP32-S3 WROOM (FREENOVE)
 *   - 5x Sensores IR HW-511 (array lejano)
 *   - 5x Sensores IR TCRT5000 (array cercano)
 *   - L298N (puente H para motores)
 *   - 2x Motores DC con reductora
 *
 * Autor: LUCHIN-OPRESORCL
 * Fecha: 2025-10-29
 * Versión: 1.5.3
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

// Estado actual del robot
EstadoRobot estadoActual = CALIBRANDO;
EstadoRobot estadoAnterior = CALIBRANDO;

// Velocidad base del robot (puede ajustarse dinámicamente)
uint8_t velocidadBase = VELOCIDAD_BASE;

// Tiempo de última detección de línea
unsigned long tiempoPerdidaLinea = 0;

// Dirección de búsqueda (true = derecha, false = izquierda)
bool direccionBusqueda = true;

// Telemetría
unsigned long tiempoUltimaTelemetria = 0;
unsigned long ciclosProcesamiento = 0;
unsigned long tiempoInicio = 0;

// Comando serial
String comandoSerial = "";

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

    // Pequeño delay para no saturar el CPU (100 Hz de actualización)
    delay(10);
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
 * Estado principal: sigue la línea usando control PID y fusión de sensores.
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

    // 4. Calcular la corrección del PID a partir del error
    // (La lógica de PID adaptativo y ajuste de velocidad por curvatura se ha eliminado
    // ya que no es aplicable con un solo array de sensores)
    float correccion = pid.calcular(error);

    // 5. Aplicar la corrección a los motores para el control diferencial
    int16_t velIzq = velocidadBase - correccion;
    int16_t velDer = velocidadBase + correccion;

    motores.diferencial(velIzq, velDer);
}

/*******************************************************************************
 * ESTADO: PERDIDA_LINEA
 *
 * La línea se perdió temporalmente. Mantiene última dirección conocida
 * durante un tiempo antes de entrar en modo búsqueda.
 ******************************************************************************/
void estadoPerdidaLinea() {
    // Leer sensores
    sensores.leer();
    sensores.procesar();

    // Verificar si se recuperó la línea
    if (sensores.isLineaDetectada()) {
        Serial.println("Línea recuperada!");
        cambiarEstado(SIGUIENDO_LINEA);
        return;
    }

    // Verificar timeout
    if (millis() - tiempoPerdidaLinea > TIMEOUT_PERDIDA_LINEA) {
        Serial.println("Timeout de pérdida. Iniciando búsqueda activa...");
        cambiarEstado(BUSCANDO_LINEA);
        return;
    }

    // Mantener última dirección con velocidad reducida
    motores.avanzar(VELOCIDAD_MIN);
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
            if (comandoSerial.length() > 0) {
                ejecutarComando(comandoSerial);
                comandoSerial = "";
            }
        } else {
            comandoSerial += c;
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
        motores.detener();
        cambiarEstado(DETENIDO);
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

    // Comando: p [Kp] [Ki] [Kd] / pid [Kp] [Ki] [Kd] - Ajustar PID
    else if (cmd.startsWith("p ") || cmd.startsWith("pid ")) {
        float kp, ki, kd;
        int n = sscanf(cmd.c_str() + (cmd.startsWith("pid ") ? 4 : 2), "%f %f %f", &kp, &ki, &kd);

        if (n == 3) {
            // Validar rangos razonables
            if (kp >= 0 && kp <= 10 && ki >= 0 && ki <= 5 && kd >= 0 && kd <= 10) {
                pid.setParametros(kp, ki, kd);
                Serial.println("✓ Parámetros PID actualizados:");
                Serial.print("  Kp="); Serial.print(kp);
                Serial.print(" | Ki="); Serial.print(ki);
                Serial.print(" | Kd="); Serial.println(kd);
                Serial.println("💾 Tip: Use 'save' para guardar en Flash");
                flagConfigChanged = true;
                guardarConfigPendiente = true;
            } else {
                Serial.println("✗ Valores fuera de rango");
                Serial.println("  Kp: 0-10, Ki: 0-5, Kd: 0-10");
            }
        } else {
            Serial.println("✗ Formato inválido");
            Serial.println("  Uso: p <Kp> <Ki> <Kd>");
            Serial.println("  Ejemplo: p 2.0 0.1 1.5");
        }
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
        Serial.print("Velocidad: "); Serial.println(velocidadBase);

        float kp, ki, kd;
        pid.getParametros(kp, ki, kd);
        Serial.print("PID: Kp="); Serial.print(kp);
        Serial.print(" Ki="); Serial.print(ki);
        Serial.print(" Kd="); Serial.println(kd);
        Serial.println("========================================");

        // Opciones según estado
        if (estadoActual == PAUSADO) {
            Serial.println("\nOpciones disponibles:");
            Serial.println("  - 'resume' para continuar");
            Serial.println("  - 'config' para configurar");
            Serial.println("  - 'p <Kp> <Ki> <Kd>' para ajustar PID");
            Serial.println("  - 'v <vel>' para cambiar velocidad");
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
        motores.detener();
        cambiarEstado(DIAGNOSTICO);
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
    Serial.println("⚙️  CONFIGURACIÓN:");
    Serial.println("  p <Kp> <Ki> <Kd>   - Ajustar parámetros PID");
    Serial.println("                       Ej: p 2.0 0.1 1.5");
    Serial.println("  v <velocidad>      - Cambiar velocidad base");
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
    Serial.println("  test               - Test básico de motores");
    Serial.println();
    Serial.println("  💡 Nota: Use comando '0' para detener tests de motores");
    Serial.println("           Use 'x' para salir de tests de sensores/PID");
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
        motores.detener();
        cambiarEstado(DETENIDO);
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
