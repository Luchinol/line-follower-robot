/*******************************************************************************
 * NVS_CONFIG.H - Gestión de Configuración Persistente
 *
 * Este archivo maneja el almacenamiento no volátil (NVS - Non-Volatile Storage)
 * de la configuración del robot en la memoria Flash del ESP32.
 *
 * Funcionalidades:
 * - Guardar parámetros PID en Flash
 * - Guardar velocidad base en Flash
 * - Cargar configuración al iniciar
 * - Restaurar valores por defecto
 *
 * CARACTERÍSTICAS NVS:
 * - Memoria Flash dedicada (no se borra al apagar)
 * - Resistente a pérdida de energía
 * - Sistema clave-valor optimizado
 * - Ciclos de escritura: ~100,000 por sector
 *
 * Autor: LUCHIN-OPRESORCL
 * Fecha: 2025-11-03
 * Versión: 1.0.0
 ******************************************************************************/

#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>  // Biblioteca NVS de ESP32

/*******************************************************************************
 * CONFIGURACIÓN NVS
 ******************************************************************************/

// Namespace para almacenamiento (máximo 15 caracteres)
#define NVS_NAMESPACE "robot_config"

// Claves para valores almacenados (máximo 15 caracteres cada una)
#define NVS_KEY_VELOCIDAD   "vel_base"
#define NVS_KEY_KP          "pid_kp"
#define NVS_KEY_KI          "pid_ki"
#define NVS_KEY_KD          "pid_kd"
#define NVS_KEY_INITIALIZED "initialized"

/*******************************************************************************
 * CLASE: ConfiguracionNVS
 *
 * Gestiona el almacenamiento y recuperación de configuración en Flash.
 ******************************************************************************/
class ConfiguracionNVS {
private:
    Preferences preferences;
    bool configuracionCargada;

public:
    /***************************************************************************
     * Constructor
     ***************************************************************************/
    ConfiguracionNVS() : configuracionCargada(false) {}

    /***************************************************************************
     * Inicializa el sistema NVS
     * Retorna: true si se inicializó correctamente
     ***************************************************************************/
    bool inicializar() {
        Serial.println("Inicializando sistema de configuración persistente (NVS)...");

        // Abrir namespace en modo lectura para verificar si existe configuración
        if (!preferences.begin(NVS_NAMESPACE, true)) {  // true = solo lectura
            Serial.println("  ✗ Error al abrir NVS");
            return false;
        }

        // Verificar si ya existe configuración guardada
        bool inicializado = preferences.getBool(NVS_KEY_INITIALIZED, false);
        preferences.end();

        if (inicializado) {
            Serial.println("  ✓ Configuración encontrada en memoria Flash");
        } else {
            Serial.println("  ⓘ Primera ejecución: usando valores por defecto");
        }

        configuracionCargada = true;
        return true;
    }

    /***************************************************************************
     * Guarda la configuración actual en NVS
     *
     * Parámetros:
     *   velocidad - Velocidad base del robot (0-255)
     *   kp, ki, kd - Parámetros del controlador PID
     *
     * Retorna: true si se guardó correctamente
     ***************************************************************************/
    bool guardarConfiguracion(uint8_t velocidad, float kp, float ki, float kd) {
        Serial.println("\n========================================");
        Serial.println("GUARDANDO CONFIGURACIÓN EN FLASH...");
        Serial.println("========================================");

        // Abrir namespace en modo escritura
        if (!preferences.begin(NVS_NAMESPACE, false)) {  // false = lectura/escritura
            Serial.println("✗ Error al abrir NVS para escritura");
            return false;
        }

        // Guardar cada valor
        preferences.putUChar(NVS_KEY_VELOCIDAD, velocidad);
        preferences.putFloat(NVS_KEY_KP, kp);
        preferences.putFloat(NVS_KEY_KI, ki);
        preferences.putFloat(NVS_KEY_KD, kd);
        preferences.putBool(NVS_KEY_INITIALIZED, true);

        // Cerrar para asegurar escritura
        preferences.end();

        Serial.println("✓ Configuración guardada exitosamente:");
        Serial.print("  Velocidad base: "); Serial.println(velocidad);
        Serial.print("  PID Kp: "); Serial.println(kp, 3);
        Serial.print("  PID Ki: "); Serial.println(ki, 3);
        Serial.print("  PID Kd: "); Serial.println(kd, 3);
        Serial.println("========================================\n");

        return true;
    }

    /***************************************************************************
     * Carga la configuración desde NVS
     *
     * Parámetros de salida (por referencia):
     *   velocidad - Velocidad base cargada
     *   kp, ki, kd - Parámetros PID cargados
     *
     * Retorna: true si había configuración guardada
     ***************************************************************************/
    bool cargarConfiguracion(uint8_t &velocidad, float &kp, float &ki, float &kd) {
        // Abrir namespace en modo lectura
        if (!preferences.begin(NVS_NAMESPACE, true)) {
            Serial.println("✗ Error al abrir NVS para lectura");
            return false;
        }

        // Verificar si hay configuración guardada
        bool inicializado = preferences.getBool(NVS_KEY_INITIALIZED, false);

        if (!inicializado) {
            preferences.end();
            Serial.println("ⓘ No hay configuración guardada, usando valores por defecto");
            return false;
        }

        // Leer valores guardados
        velocidad = preferences.getUChar(NVS_KEY_VELOCIDAD, VELOCIDAD_BASE);
        kp = preferences.getFloat(NVS_KEY_KP, PID_RECTA.Kp);
        ki = preferences.getFloat(NVS_KEY_KI, PID_RECTA.Ki);
        kd = preferences.getFloat(NVS_KEY_KD, PID_RECTA.Kd);

        preferences.end();

        Serial.println("\n========================================");
        Serial.println("CONFIGURACIÓN CARGADA DESDE FLASH");
        Serial.println("========================================");
        Serial.print("  Velocidad base: "); Serial.println(velocidad);
        Serial.print("  PID Kp: "); Serial.println(kp, 3);
        Serial.print("  PID Ki: "); Serial.println(ki, 3);
        Serial.print("  PID Kd: "); Serial.println(kd, 3);
        Serial.println("========================================\n");

        return true;
    }

    /***************************************************************************
     * Borra toda la configuración guardada y restaura valores por defecto
     *
     * Retorna: true si se borró correctamente
     ***************************************************************************/
    bool restaurarDefecto() {
        Serial.println("\n========================================");
        Serial.println("RESTAURANDO VALORES POR DEFECTO...");
        Serial.println("========================================");

        // Abrir namespace en modo escritura
        if (!preferences.begin(NVS_NAMESPACE, false)) {
            Serial.println("✗ Error al abrir NVS");
            return false;
        }

        // Borrar todos los valores
        preferences.clear();
        preferences.end();

        Serial.println("✓ Configuración borrada");
        Serial.println("  En el próximo reinicio se usarán valores por defecto");
        Serial.println("========================================\n");

        return true;
    }

    /***************************************************************************
     * Verifica si existe configuración guardada
     *
     * Retorna: true si hay configuración guardada
     ***************************************************************************/
    bool existeConfiguracion() {
        if (!preferences.begin(NVS_NAMESPACE, true)) {
            return false;
        }

        bool inicializado = preferences.getBool(NVS_KEY_INITIALIZED, false);
        preferences.end();

        return inicializado;
    }

    /***************************************************************************
     * Muestra información sobre el espacio usado en NVS
     ***************************************************************************/
    void mostrarInfoNVS() {
        Serial.println("\n========================================");
        Serial.println("INFORMACIÓN DE ALMACENAMIENTO NVS");
        Serial.println("========================================");

        if (!preferences.begin(NVS_NAMESPACE, true)) {
            Serial.println("✗ Error al abrir NVS");
            return;
        }

        bool inicializado = preferences.getBool(NVS_KEY_INITIALIZED, false);

        Serial.print("Estado: ");
        if (inicializado) {
            Serial.println("Configuración guardada");

            Serial.println("\nValores almacenados:");
            Serial.print("  Velocidad: ");
            Serial.println(preferences.getUChar(NVS_KEY_VELOCIDAD, 0));
            Serial.print("  PID Kp: ");
            Serial.println(preferences.getFloat(NVS_KEY_KP, 0.0), 3);
            Serial.print("  PID Ki: ");
            Serial.println(preferences.getFloat(NVS_KEY_KI, 0.0), 3);
            Serial.print("  PID Kd: ");
            Serial.println(preferences.getFloat(NVS_KEY_KD, 0.0), 3);
        } else {
            Serial.println("Sin configuración guardada");
        }

        preferences.end();

        Serial.println("\nCapacidades NVS:");
        Serial.println("  Namespace: " NVS_NAMESPACE);
        Serial.println("  Persistencia: Sobrevive apagados y reinicios");
        Serial.println("  Ciclos de escritura: ~100,000 por sector");
        Serial.println("========================================\n");
    }
};

#endif // NVS_CONFIG_H
