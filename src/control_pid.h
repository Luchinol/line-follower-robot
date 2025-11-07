/*******************************************************************************
 * CONTROL_PID.H - Controlador PID Adaptativo
 *
 * Implementa un controlador PID (Proporcional-Integral-Derivativo) adaptativo
 * que ajusta sus parámetros según la curvatura de la trayectoria.
 *
 * Ecuación PID:
 *   u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt
 *
 * Donde:
 *   - e(t): Error actual (desviación de la línea)
 *   - Kp: Ganancia proporcional (respuesta al error actual)
 *   - Ki: Ganancia integral (corrige error acumulado)
 *   - Kd: Ganancia derivativa (anticipa cambios)
 *
 * Características:
 *   - Adaptación automática según curvatura (recta/curva suave/curva cerrada)
 *   - Anti-windup para la integral (evita saturación)
 *   - Filtro derivativo para reducir ruido
 *   - Ajuste dinámico de parámetros
 *   - Reset automático en cambios bruscos
 *
 * Autor: LUCHIN-OPRESORCL
 * Fecha: 2025-11-07
 * Versión: 2.0.0
 ******************************************************************************/

#ifndef CONTROL_PID_H
#define CONTROL_PID_H

#include "config.h"

/*******************************************************************************
 * CLASE: ControladorPID
 *
 * Implementa un controlador PID adaptativo con las siguientes características:
 * - Ajuste automático de parámetros según curvatura
 * - Anti-windup (limita acumulación de error integral)
 * - Filtro de derivada para reducir ruido
 * - Estadísticas y telemetría
 ******************************************************************************/
class ControladorPID {
private:
    // Parámetros PID actuales
    float Kp;  // Ganancia Proporcional
    float Ki;  // Ganancia Integral
    float Kd;  // Ganancia Derivativa

    // Variables de estado del PID
    float errorAnterior;      // Error en la iteración anterior
    float integral;           // Suma acumulada de errores
    float derivada;           // Tasa de cambio del error

    // Términos calculados (para debug/monitoreo)
    float terminoP;           // Último término proporcional calculado
    float terminoI;           // Último término integral calculado
    float terminoD;           // Último término derivativo calculado

    // Tiempo
    unsigned long tiempoAnterior;  // Tiempo de la última actualización (ms)
    float dt;                      // Delta de tiempo entre actualizaciones (s)

    // Límites anti-windup
    float limiteIntegral;     // Límite máximo para la integral
    float limiteSalida;       // Límite máximo para la salida total

    // Filtro de derivada (suavizado exponencial)
    float alfaDerivada;       // Factor de suavizado (0-1, típico 0.1-0.3)
    float derivadaFiltrada;   // Derivada filtrada

    // Modo de operación actual
    enum ModoPID {
        MODO_RECTA,
        MODO_CURVA_SUAVE,
        MODO_CURVA_CERRADA
    };
    ModoPID modoActual;

    // Estadísticas
    float errorPromedio;      // Error promedio (para telemetría)
    float errorMaximo;        // Error máximo observado
    uint32_t ciclosPID;       // Contador de ciclos

    // Flag de inicialización
    bool inicializado;

    /***************************************************************************
     * Aplica el límite anti-windup a la integral
     *
     * Evita que la integral crezca sin control cuando el robot no puede
     * seguir la línea (saturación del actuador)
     ***************************************************************************/
    void aplicarAntiWindup() {
        if (integral > limiteIntegral) {
            integral = limiteIntegral;
        } else if (integral < -limiteIntegral) {
            integral = -limiteIntegral;
        }
    }

    /***************************************************************************
     * Aplica filtro de suavizado exponencial a la derivada
     *
     * Reduce el ruido en la señal derivativa:
     *   derivada_filtrada = α·derivada_nueva + (1-α)·derivada_anterior
     *
     * Parámetros:
     *   derivadaNueva: Valor de derivada calculado
     ***************************************************************************/
    void filtrarDerivada(float derivadaNueva) {
        derivadaFiltrada = alfaDerivada * derivadaNueva +
                          (1.0 - alfaDerivada) * derivadaFiltrada;
    }

public:
    /***************************************************************************
     * Constructor - Inicializa el controlador PID
     *
     * Parámetros:
     *   kp: Ganancia proporcional inicial
     *   ki: Ganancia integral inicial
     *   kd: Ganancia derivativa inicial
     ***************************************************************************/
    ControladorPID(float kp = 1.5, float ki = 0.05, float kd = 0.8) {
        Kp = kp;
        Ki = ki;
        Kd = kd;

        errorAnterior = 0;
        integral = 0;
        derivada = 0;
        derivadaFiltrada = 0;

        tiempoAnterior = 0;
        dt = 0;

        limiteIntegral = 1000.0;    // Límite de integral
        limiteSalida = CORRECCION_MAX;  // Límite de salida (de config.h)

        alfaDerivada = 0.2;  // Factor de suavizado (20% nueva, 80% anterior)

        modoActual = MODO_RECTA;

        errorPromedio = 0;
        errorMaximo = 0;
        ciclosPID = 0;

        inicializado = false;
    }

    /***************************************************************************
     * Inicializa el controlador PID
     ***************************************************************************/
    void inicializar() {
        DEBUG_PRINTLN(DEBUG_PID, "Inicializando controlador PID...");
        DEBUG_PRINT(DEBUG_PID, "  Kp: "); DEBUG_PRINTLN(DEBUG_PID, Kp);
        DEBUG_PRINT(DEBUG_PID, "  Ki: "); DEBUG_PRINTLN(DEBUG_PID, Ki);
        DEBUG_PRINT(DEBUG_PID, "  Kd: "); DEBUG_PRINTLN(DEBUG_PID, Kd);

        reset();
        inicializado = true;

        DEBUG_PRINTLN(DEBUG_PID, "PID inicializado correctamente\n");
    }

    /***************************************************************************
     * Reinicia el estado del controlador PID
     *
     * Útil al:
     * - Perder la línea
     * - Cambiar de modo
     * - Detectar un cambio brusco
     ***************************************************************************/
    void reset() {
        errorAnterior = 0;
        integral = 0;
        derivada = 0;
        derivadaFiltrada = 0;
        tiempoAnterior = millis();

        DEBUG_PRINTLN(DEBUG_PID, "PID reseteado");
    }

    /***************************************************************************
     * Ajusta los parámetros PID según la curvatura detectada
     *
     * Parámetros:
     *   curvatura: Valor de curvatura (0-800 típico)
     *
     * Estrategias:
     *   - RECTA (curvatura < 50):
     *       • Control suave y estable
     *       • Usa integral para corregir error persistente
     *
     *   - CURVA_SUAVE (50 ≤ curvatura < 150):
     *       • Mayor respuesta proporcional
     *       • Más anticipación con derivada
     *
     *   - CURVA_CERRADA (curvatura ≥ 150):
     *       • Máxima agresividad
     *       • Sin integral (evita wind-up en curvas)
     *       • Máxima derivada para anticipar
     ***************************************************************************/
    void ajustarParametros(uint16_t curvatura) {
        ModoPID modoNuevo;

        // Determinar modo según curvatura
        if (curvatura < UMBRAL_CURVA_SUAVE) {
            // RECTA
            modoNuevo = MODO_RECTA;
            Kp = PID_RECTA.Kp;
            Ki = PID_RECTA.Ki;
            Kd = PID_RECTA.Kd;
        } else if (curvatura < UMBRAL_CURVA_CERRADA) {
            // CURVA SUAVE
            modoNuevo = MODO_CURVA_SUAVE;
            Kp = PID_CURVA_SUAVE.Kp;
            Ki = PID_CURVA_SUAVE.Ki;
            Kd = PID_CURVA_SUAVE.Kd;
        } else {
            // CURVA CERRADA
            modoNuevo = MODO_CURVA_CERRADA;
            Kp = PID_CURVA_CERRADA.Kp;
            Ki = PID_CURVA_CERRADA.Ki;
            Kd = PID_CURVA_CERRADA.Kd;
        }

        // Si cambió el modo, resetear integral para evitar transitorios
        if (modoNuevo != modoActual) {
            integral = 0;  // Reset solo de integral, no de toda la historia
            modoActual = modoNuevo;

            DEBUG_PRINT(DEBUG_PID, "Cambio de modo PID: ");
            switch (modoActual) {
                case MODO_RECTA:
                    DEBUG_PRINTLN(DEBUG_PID, "RECTA");
                    break;
                case MODO_CURVA_SUAVE:
                    DEBUG_PRINTLN(DEBUG_PID, "CURVA_SUAVE");
                    break;
                case MODO_CURVA_CERRADA:
                    DEBUG_PRINTLN(DEBUG_PID, "CURVA_CERRADA");
                    break;
            }
            DEBUG_PRINT(DEBUG_PID, "  Nuevos parámetros -> Kp: ");
            DEBUG_PRINT(DEBUG_PID, Kp);
            DEBUG_PRINT(DEBUG_PID, " Ki: ");
            DEBUG_PRINT(DEBUG_PID, Ki);
            DEBUG_PRINT(DEBUG_PID, " Kd: ");
            DEBUG_PRINTLN(DEBUG_PID, Kd);
        }
    }

    /***************************************************************************
     * Calcula la salida del controlador PID
     *
     * Parámetros:
     *   error: Error actual (desviación de la línea, -400 a +400)
     *
     * Retorna: Corrección a aplicar (-CORRECCION_MAX a +CORRECCION_MAX)
     *
     * Proceso:
     *   1. Calcular dt (tiempo transcurrido)
     *   2. Calcular término proporcional: P = Kp * error
     *   3. Calcular término integral: I = Ki * Σ(error * dt)
     *   4. Calcular término derivativo: D = Kd * (error - error_anterior) / dt
     *   5. Sumar: salida = P + I + D
     *   6. Limitar salida al rango permitido
     ***************************************************************************/
    float calcular(float error) {
        // Calcular tiempo transcurrido
        unsigned long tiempoActual = millis();
        dt = (tiempoActual - tiempoAnterior) / 1000.0;  // Convertir a segundos

        // Protección: si dt es muy pequeño o muy grande, usar valor por defecto
        if (dt <= 0 || dt > 1.0) {
            dt = 0.01;  // 10ms por defecto
        }

        // ===== TÉRMINO PROPORCIONAL =====
        // Respuesta directamente proporcional al error actual
        terminoP = Kp * error;

        // ===== TÉRMINO INTEGRAL =====
        // Acumula el error en el tiempo (corrige error persistente)
        integral += error * dt;
        aplicarAntiWindup();  // Limitar crecimiento excesivo
        terminoI = Ki * integral;

        // ===== TÉRMINO DERIVATIVO =====
        // Anticipa cambios futuros basándose en la tasa de cambio
        float derivadaNueva = (error - errorAnterior) / dt;
        filtrarDerivada(derivadaNueva);  // Suavizar para reducir ruido
        terminoD = Kd * derivadaFiltrada;

        // ===== SALIDA TOTAL =====
        float salida = terminoP + terminoI + terminoD;

        // Limitar salida al rango permitido
        if (salida > limiteSalida) {
            salida = limiteSalida;
        } else if (salida < -limiteSalida) {
            salida = -limiteSalida;
        }

        // Actualizar valores para siguiente iteración
        errorAnterior = error;
        tiempoAnterior = tiempoActual;

        // Actualizar estadísticas
        ciclosPID++;
        errorPromedio = (errorPromedio * 0.95) + (abs(error) * 0.05);  // Media móvil
        if (abs(error) > errorMaximo) {
            errorMaximo = abs(error);
        }

        // Debug detallado
        DEBUG_PRINT(DEBUG_PID, "PID: Error=");
        DEBUG_PRINT(DEBUG_PID, error);
        DEBUG_PRINT(DEBUG_PID, " | P=");
        DEBUG_PRINT(DEBUG_PID, terminoP);
        DEBUG_PRINT(DEBUG_PID, " I=");
        DEBUG_PRINT(DEBUG_PID, terminoI);
        DEBUG_PRINT(DEBUG_PID, " D=");
        DEBUG_PRINT(DEBUG_PID, terminoD);
        DEBUG_PRINT(DEBUG_PID, " | Salida=");
        DEBUG_PRINTLN(DEBUG_PID, salida);

        return salida;
    }

    /***************************************************************************
     * Establece manualmente los parámetros PID
     *
     * Útil para ajuste fino durante pruebas
     ***************************************************************************/
    void setParametros(float kp, float ki, float kd) {
        Kp = kp;
        Ki = ki;
        Kd = kd;

        Serial.println("\nParámetros PID actualizados:");
        Serial.print("  Kp: "); Serial.println(Kp);
        Serial.print("  Ki: "); Serial.println(Ki);
        Serial.print("  Kd: "); Serial.println(Kd);
    }

    /***************************************************************************
     * Obtiene los parámetros PID actuales
     ***************************************************************************/
    void getParametros(float& kp, float& ki, float& kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }

    /***************************************************************************
     * Obtiene el último término proporcional calculado (para debug)
     ***************************************************************************/
    float obtenerP() {
        return terminoP;
    }

    /***************************************************************************
     * Obtiene el último término integral calculado (para debug)
     ***************************************************************************/
    float obtenerI() {
        return terminoI;
    }

    /***************************************************************************
     * Obtiene el último término derivativo calculado (para debug)
     ***************************************************************************/
    float obtenerD() {
        return terminoD;
    }

    /***************************************************************************
     * Establece el límite de salida del PID
     ***************************************************************************/
    void setLimiteSalida(float limite) {
        limiteSalida = limite;
    }

    /***************************************************************************
     * Establece el límite anti-windup de la integral
     ***************************************************************************/
    void setLimiteIntegral(float limite) {
        limiteIntegral = limite;
    }

    /***************************************************************************
     * Obtiene el modo PID actual
     ***************************************************************************/
    const char* obtenerModoActual() {
        switch (modoActual) {
            case MODO_RECTA:         return "RECTA";
            case MODO_CURVA_SUAVE:   return "CURVA_SUAVE";
            case MODO_CURVA_CERRADA: return "CURVA_CERRADA";
            default:                 return "DESCONOCIDO";
        }
    }

    /***************************************************************************
     * Obtiene el error promedio (para telemetría)
     ***************************************************************************/
    float obtenerErrorPromedio() {
        return errorPromedio;
    }

    /***************************************************************************
     * Obtiene el error máximo observado
     ***************************************************************************/
    float obtenerErrorMaximo() {
        return errorMaximo;
    }

    /***************************************************************************
     * Obtiene el número de ciclos ejecutados
     ***************************************************************************/
    uint32_t obtenerCiclos() {
        return ciclosPID;
    }

    /***************************************************************************
     * Obtiene el valor actual de la integral
     ***************************************************************************/
    float obtenerIntegral() {
        return integral;
    }

    /***************************************************************************
     * Imprime el estado completo del PID
     ***************************************************************************/
    void imprimirEstado() {
        Serial.println("\n========== ESTADO DEL PID ==========");
        Serial.print("Modo actual:      "); Serial.println(obtenerModoActual());
        Serial.print("Parámetros:       Kp="); Serial.print(Kp);
        Serial.print(" Ki="); Serial.print(Ki);
        Serial.print(" Kd="); Serial.println(Kd);
        Serial.println();
        Serial.print("Error anterior:   "); Serial.println(errorAnterior);
        Serial.print("Integral:         "); Serial.println(integral);
        Serial.print("Derivada (filt):  "); Serial.println(derivadaFiltrada);
        Serial.println();
        Serial.print("Error promedio:   "); Serial.println(errorPromedio);
        Serial.print("Error máximo:     "); Serial.println(errorMaximo);
        Serial.print("Ciclos PID:       "); Serial.println(ciclosPID);
        Serial.println("====================================\n");
    }

    /***************************************************************************
     * Reinicia estadísticas (útil para nuevas pruebas)
     ***************************************************************************/
    void reiniciarEstadisticas() {
        errorPromedio = 0;
        errorMaximo = 0;
        ciclosPID = 0;
        Serial.println("Estadísticas del PID reiniciadas");
    }
};

#endif // CONTROL_PID_H
