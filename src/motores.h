/*******************************************************************************
 * MOTORES.H - Control de Motores DC con L298N
 *
 * Este archivo maneja el control de dos motores DC mediante el puente H L298N:
 * - Configuración de pines y canales PWM del ESP32
 * - Control de velocidad mediante PWM (0-255)
 * - Control de dirección (adelante/atrás)
 * - Control diferencial (velocidades independientes para giros)
 * - Funciones de movimiento básico (avanzar, retroceder, girar, detener)
 * - Protecciones y límites de velocidad
 *
 * Conexiones L298N:
 * - ENA/ENB: Pines PWM del ESP32 para control de velocidad
 * - IN1/IN2: Pines GPIO para dirección del motor derecho
 * - IN3/IN4: Pines GPIO para dirección del motor izquierdo
 *
 * Autor: LUCHIN-OPRESORCL
 * Fecha: 2025-11-07
 * Versión: 2.0.0
 ******************************************************************************/

#ifndef MOTORES_H
#define MOTORES_H

#include "config.h"

/*******************************************************************************
 * CLASE: ControlMotores
 *
 * Gestiona el control de dos motores DC mediante el puente H L298N.
 * Proporciona funciones de alto nivel para movimiento y control diferencial.
 ******************************************************************************/
class ControlMotores {
private:
    // Velocidades actuales de cada motor (0-255)
    uint8_t velocidadActualDerecho = 0;
    uint8_t velocidadActualIzquierdo = 0;

    // Dirección actual de cada motor (true = adelante, false = atrás)
    bool direccionDerecho = true;
    bool direccionIzquierdo = true;

    // Flags de inicialización
    bool inicializado = false;

    // Estadísticas
    unsigned long distanciaRecorrida = 0;  // En milímetros (estimado)
    unsigned long tiempoMovimiento = 0;    // En milisegundos

    /***************************************************************************
     * Mapea velocidad lógica (0-255) a PWM efectivo (102-255)
     *
     * ¿POR QUÉ ES NECESARIO ESTE MAPEO?
     * ─────────────────────────────────
     * Los motores DC con driver L298N tienen una "zona muerta" (dead zone)
     * entre 0-40% PWM donde:
     *   ✗ El torque generado < fricción estática del motor
     *   ✗ Los motores vibran pero NO giran
     *   ✗ El comportamiento es impredecible y no lineal
     *   ✗ Desperdicio de energía sin movimiento útil
     *
     * SOLUCIÓN IMPLEMENTADA:
     * ───────────────────────
     * Mapeamos el rango de usuario (0-255) al rango útil real (40%-100% PWM)
     *
     *   Velocidad Usuario  →  PWM Real (12V)  →  Comportamiento
     *   ─────────────────     ──────────────     ──────────────
     *         0                    0 (0%)         Motor detenido
     *         1                  102 (40%)        Velocidad mínima confiable
     *       128                  178 (70%)        Velocidad media
     *       255                  255 (100%)       Velocidad máxima
     *
     * FÓRMULA DE MAPEO:
     * ─────────────────
     *   PWM_real = PWM_MIN_EFECTIVO +
     *              (velocidad - 1) × (PWM_MAX - PWM_MIN) / 254
     *
     * VALORES DEL PROYECTO:
     *   PWM_MIN_EFECTIVO = 102 (40% de 255)
     *   PWM_MAX_EFECTIVO = 255 (100%)
     *
     * RESULTADO:
     *   ✓ Control lineal y predecible en todo el rango
     *   ✓ Eliminación completa de la zona muerta
     *   ✓ Mayor precisión en velocidades bajas
     *   ✓ Respuesta consistente del motor
     *
     * Parámetros:
     *   velocidad: Velocidad lógica (0-255)
     *
     * Retorna: PWM real mapeado al rango efectivo (0 o 102-255)
     ***************************************************************************/
    uint8_t mapearVelocidad(uint8_t velocidad) {
        if (velocidad == 0) return 0;  // Detenido = 0 PWM real

        // Mapear linealmente: 1-255 → PWM_MIN_EFECTIVO-PWM_MAX_EFECTIVO
        long rango_entrada = 255 - 1;  // 254
        long rango_salida = PWM_MAX_EFECTIVO - PWM_MIN_EFECTIVO;  // 153
        long pwm = PWM_MIN_EFECTIVO + ((velocidad - 1) * rango_salida) / rango_entrada;

        return (uint8_t)pwm;
    }

    /***************************************************************************
     * Limita un valor de velocidad al rango permitido
     *
     * Parámetros:
     *   velocidad: Velocidad deseada (-255 a +255)
     *
     * Retorna: Velocidad limitada al rango válido
     ***************************************************************************/
    int16_t limitarVelocidad(int16_t velocidad) {
        // Aplicar velocidad mínima si es muy baja
        if (abs(velocidad) > 0 && abs(velocidad) < VELOCIDAD_MIN) {
            velocidad = (velocidad > 0) ? VELOCIDAD_MIN : -VELOCIDAD_MIN;
        }

        // Limitar al rango permitido
        if (velocidad > VELOCIDAD_MAX) velocidad = VELOCIDAD_MAX;
        if (velocidad < -VELOCIDAD_MAX) velocidad = -VELOCIDAD_MAX;

        return velocidad;
    }

public:
    /***************************************************************************
     * Constructor
     ***************************************************************************/
    ControlMotores() {}

    /***************************************************************************
     * Inicializa los pines de control de motores y configura PWM
     ***************************************************************************/
    void inicializar() {
        DEBUG_PRINTLN(DEBUG_MOTORES, "Inicializando control de motores...");

        // ===== MOTOR DERECHO =====
        // Configurar pines de dirección como salida
        pinMode(MOTOR_DER_IN1, OUTPUT);
        pinMode(MOTOR_DER_IN2, OUTPUT);
        pinMode(MOTOR_DER_ENA, OUTPUT);

        DEBUG_PRINTLN(DEBUG_MOTORES, "  Motor Derecho:");
        DEBUG_PRINT(DEBUG_MOTORES, "    ENA (PWM): GPIO ");
        DEBUG_PRINTLN(DEBUG_MOTORES, MOTOR_DER_ENA);
        DEBUG_PRINT(DEBUG_MOTORES, "    IN1: GPIO ");
        DEBUG_PRINTLN(DEBUG_MOTORES, MOTOR_DER_IN1);
        DEBUG_PRINT(DEBUG_MOTORES, "    IN2: GPIO ");
        DEBUG_PRINTLN(DEBUG_MOTORES, MOTOR_DER_IN2);

        // Configurar canal PWM para motor derecho
        // ledcSetup(canal, frecuencia, resolución)
        ledcSetup(PWM_CHANNEL_DER, PWM_FREQUENCY, PWM_RESOLUTION);
        // ledcAttachPin(pin_gpio, canal)
        ledcAttachPin(MOTOR_DER_ENA, PWM_CHANNEL_DER);

        // ===== MOTOR IZQUIERDO =====
        // Configurar pines de dirección como salida
        pinMode(MOTOR_IZQ_IN3, OUTPUT);
        pinMode(MOTOR_IZQ_IN4, OUTPUT);
        pinMode(MOTOR_IZQ_ENB, OUTPUT);

        DEBUG_PRINTLN(DEBUG_MOTORES, "  Motor Izquierdo:");
        DEBUG_PRINT(DEBUG_MOTORES, "    ENB (PWM): GPIO ");
        DEBUG_PRINTLN(DEBUG_MOTORES, MOTOR_IZQ_ENB);
        DEBUG_PRINT(DEBUG_MOTORES, "    IN3: GPIO ");
        DEBUG_PRINTLN(DEBUG_MOTORES, MOTOR_IZQ_IN3);
        DEBUG_PRINT(DEBUG_MOTORES, "    IN4: GPIO ");
        DEBUG_PRINTLN(DEBUG_MOTORES, MOTOR_IZQ_IN4);

        // Configurar canal PWM para motor izquierdo
        ledcSetup(PWM_CHANNEL_IZQ, PWM_FREQUENCY, PWM_RESOLUTION);
        ledcAttachPin(MOTOR_IZQ_ENB, PWM_CHANNEL_IZQ);

        // Detener motores inicialmente
        detener();

        inicializado = true;
        DEBUG_PRINTLN(DEBUG_MOTORES, "Motores inicializados correctamente\n");
    }

    /***************************************************************************
     * Establece la velocidad y dirección del motor DERECHO
     *
     * Parámetros:
     *   velocidad: Velocidad del motor (-255 a +255)
     *              Positivo = adelante, Negativo = atrás, 0 = detenido
     *
     * NOTA: Motor derecho físico conectado a OUT3/OUT4 del L298N (ENB/IN3/IN4)
     *
     * Conexión L298N Motor Derecho:
     *   IN3 = HIGH, IN4 = LOW  → Giro adelante
     *   IN3 = LOW,  IN4 = HIGH → Giro atrás
     *   IN3 = LOW,  IN4 = LOW  → Freno activo
     *   ENB = PWM              → Control de velocidad
     ***************************************************************************/
    void setMotorDerecho(int16_t velocidad) {
        // Limitar velocidad
        velocidad = limitarVelocidad(velocidad);

        // Determinar dirección
        bool adelante = (velocidad >= 0);
        uint8_t velAbs = abs(velocidad);

        // Aplicar factor de compensación para equalizar motores
        velAbs = (uint8_t)(velAbs * FACTOR_MOTOR_DERECHO);
        velAbs = constrain(velAbs, 0, 255);

        // Mapear velocidad lógica a PWM efectivo (40%-100%)
        uint8_t pwmReal = mapearVelocidad(velAbs);

        // Configurar dirección
        if (adelante) {
            digitalWrite(MOTOR_DER_IN1, HIGH);
            digitalWrite(MOTOR_DER_IN2, LOW);
            direccionDerecho = true;
        } else {
            digitalWrite(MOTOR_DER_IN1, LOW);
            digitalWrite(MOTOR_DER_IN2, HIGH);
            direccionDerecho = false;
        }

        // Aplicar velocidad mapeada mediante PWM
        ledcWrite(PWM_CHANNEL_DER, pwmReal);
        velocidadActualDerecho = velAbs;  // Guardar velocidad lógica

        DEBUG_PRINT(DEBUG_MOTORES, "Motor Derecho: ");
        DEBUG_PRINT(DEBUG_MOTORES, adelante ? "ADELANTE" : "ATRÁS");
        DEBUG_PRINT(DEBUG_MOTORES, " | VEL: ");
        DEBUG_PRINT(DEBUG_MOTORES, velAbs);
        DEBUG_PRINT(DEBUG_MOTORES, " → PWM: ");
        DEBUG_PRINTLN(DEBUG_MOTORES, pwmReal);
    }

    /***************************************************************************
     * Establece la velocidad y dirección del motor IZQUIERDO
     *
     * Parámetros:
     *   velocidad: Velocidad del motor (-255 a +255)
     *              Positivo = adelante, Negativo = atrás, 0 = detenido
     *
     * Conexión L298N Motor Izquierdo:
     *   IN3 = HIGH, IN4 = LOW  → Giro adelante
     *   IN3 = LOW,  IN4 = HIGH → Giro atrás
     *   IN3 = LOW,  IN4 = LOW  → Freno activo
     *   ENB = PWM              → Control de velocidad
     ***************************************************************************/
    void setMotorIzquierdo(int16_t velocidad) {
        // Limitar velocidad
        velocidad = limitarVelocidad(velocidad);

        // Determinar dirección
        bool adelante = (velocidad >= 0);
        uint8_t velAbs = abs(velocidad);

        // Aplicar factor de compensación para equalizar motores
        velAbs = (uint8_t)(velAbs * FACTOR_MOTOR_IZQUIERDO);
        velAbs = constrain(velAbs, 0, 255);

        // Mapear velocidad lógica a PWM efectivo (40%-100%)
        uint8_t pwmReal = mapearVelocidad(velAbs);

        // Configurar dirección
        if (adelante) {
            digitalWrite(MOTOR_IZQ_IN3, HIGH);
            digitalWrite(MOTOR_IZQ_IN4, LOW);
            direccionIzquierdo = true;
        } else {
            digitalWrite(MOTOR_IZQ_IN3, LOW);
            digitalWrite(MOTOR_IZQ_IN4, HIGH);
            direccionIzquierdo = false;
        }

        // Aplicar velocidad mapeada mediante PWM
        ledcWrite(PWM_CHANNEL_IZQ, pwmReal);
        velocidadActualIzquierdo = velAbs;  // Guardar velocidad lógica

        DEBUG_PRINT(DEBUG_MOTORES, "Motor Izquierdo: ");
        DEBUG_PRINT(DEBUG_MOTORES, adelante ? "ADELANTE" : "ATRÁS");
        DEBUG_PRINT(DEBUG_MOTORES, " | VEL: ");
        DEBUG_PRINT(DEBUG_MOTORES, velAbs);
        DEBUG_PRINT(DEBUG_MOTORES, " → PWM: ");
        DEBUG_PRINTLN(DEBUG_MOTORES, pwmReal);
    }

    /***************************************************************************
     * Control diferencial: establece velocidades independientes
     *
     * Esta es la función principal para seguimiento de línea. Permite que
     * cada rueda gire a diferente velocidad para realizar giros suaves.
     *
     * Parámetros:
     *   velocidadIzq: Velocidad del motor izquierdo (-255 a +255)
     *   velocidadDer: Velocidad del motor derecho (-255 a +255)
     *
     * Ejemplo de uso:
     *   diferencial(200, 150);  // Giro suave a la derecha
     *   diferencial(150, 200);  // Giro suave a la izquierda
     *   diferencial(180, 180);  // Avanzar recto
     ***************************************************************************/
    void diferencial(int16_t velocidadIzq, int16_t velocidadDer) {
        setMotorIzquierdo(velocidadIzq);
        setMotorDerecho(velocidadDer);

        DEBUG_PRINT(DEBUG_MOTORES, "Diferencial -> Izq: ");
        DEBUG_PRINT(DEBUG_MOTORES, velocidadIzq);
        DEBUG_PRINT(DEBUG_MOTORES, " | Der: ");
        DEBUG_PRINTLN(DEBUG_MOTORES, velocidadDer);
    }

    /***************************************************************************
     * Giro en pivote con control directo de PWM (bypasea mapeo 40%-100%)
     *
     * Esta función permite velocidades por debajo del PWM_MIN_EFECTIVO (40%)
     * para ejecutar giros en pivote en curvas extremadamente cerradas.
     *
     * Parámetros:
     *   velocidadInterior: Velocidad rueda interior (0-100%)
     *   velocidadExterior: Velocidad rueda exterior (0-100%)
     *   giroIzquierda: true = gira a la izquierda, false = gira a la derecha
     *
     * IMPORTANTE: Esta función NO aplica el mapeo PWM automático.
     * Los valores se convierten directamente a PWM (0-255).
     *
     * Ejemplo:
     *   pivote(20, 100, true)  → Giro izquierda: Izq=20%, Der=100%
     *   pivote(20, 100, false) → Giro derecha: Izq=100%, Der=20%
     ***************************************************************************/
    void pivote(uint8_t velocidadInterior, uint8_t velocidadExterior, bool giroIzquierda) {
        // Convertir porcentajes (0-100) a PWM (0-255)
        uint8_t pwmInterior = map(velocidadInterior, 0, 100, 0, 255);
        uint8_t pwmExterior = map(velocidadExterior, 0, 100, 0, 255);

        if (giroIzquierda) {
            // Giro a la izquierda: Rueda izquierda lenta, derecha rápida
            // Motor Izquierdo (interior) - velocidad reducida
            digitalWrite(MOTOR_IZQ_IN3, HIGH);
            digitalWrite(MOTOR_IZQ_IN4, LOW);
            ledcWrite(PWM_CHANNEL_IZQ, pwmInterior);

            // Motor Derecho (exterior) - velocidad máxima
            digitalWrite(MOTOR_DER_IN1, HIGH);
            digitalWrite(MOTOR_DER_IN2, LOW);
            ledcWrite(PWM_CHANNEL_DER, pwmExterior);

            velocidadActualIzquierdo = velocidadInterior;  // Guardar valor lógico
            velocidadActualDerecho = velocidadExterior;
        } else {
            // Giro a la derecha: Rueda derecha lenta, izquierda rápida
            // Motor Izquierdo (exterior) - velocidad máxima
            digitalWrite(MOTOR_IZQ_IN3, HIGH);
            digitalWrite(MOTOR_IZQ_IN4, LOW);
            ledcWrite(PWM_CHANNEL_IZQ, pwmExterior);

            // Motor Derecho (interior) - velocidad reducida
            digitalWrite(MOTOR_DER_IN1, HIGH);
            digitalWrite(MOTOR_DER_IN2, LOW);
            ledcWrite(PWM_CHANNEL_DER, pwmInterior);

            velocidadActualIzquierdo = velocidadExterior;
            velocidadActualDerecho = velocidadInterior;  // Guardar valor lógico
        }

        DEBUG_PRINT(DEBUG_MOTORES, "PIVOTE -> ");
        DEBUG_PRINT(DEBUG_MOTORES, giroIzquierda ? "IZQ" : "DER");
        DEBUG_PRINT(DEBUG_MOTORES, " | Interior: ");
        DEBUG_PRINT(DEBUG_MOTORES, velocidadInterior);
        DEBUG_PRINT(DEBUG_MOTORES, "% | Exterior: ");
        DEBUG_PRINT(DEBUG_MOTORES, velocidadExterior);
        DEBUG_PRINTLN(DEBUG_MOTORES, "%");
    }

    /***************************************************************************
     * Avanza con ambos motores a la misma velocidad
     *
     * Parámetros:
     *   velocidad: Velocidad de avance (0 a 255)
     ***************************************************************************/
    void avanzar(uint8_t velocidad) {
        velocidad = limitarVelocidad(velocidad);
        setMotorDerecho(velocidad);
        setMotorIzquierdo(velocidad);

        DEBUG_PRINT(DEBUG_MOTORES, "Avanzar -> Velocidad: ");
        DEBUG_PRINTLN(DEBUG_MOTORES, velocidad);
    }

    /***************************************************************************
     * Retrocede con ambos motores a la misma velocidad
     *
     * Parámetros:
     *   velocidad: Velocidad de retroceso (0 a 255)
     ***************************************************************************/
    void retroceder(uint8_t velocidad) {
        velocidad = limitarVelocidad(velocidad);
        setMotorDerecho(-velocidad);
        setMotorIzquierdo(-velocidad);

        DEBUG_PRINT(DEBUG_MOTORES, "Retroceder -> Velocidad: ");
        DEBUG_PRINTLN(DEBUG_MOTORES, velocidad);
    }

    /***************************************************************************
     * Retrocede con giro (retroceso inteligente para recuperar línea)
     *
     * Retrocede girando hacia el lado donde se perdió la línea,
     * útil para recuperar la pista cuando el robot se pasa de la línea.
     *
     * Parámetros:
     *   velocidad: Velocidad de retroceso (0-255)
     *   factorGiro: Factor de diferencia entre ruedas (0.5-1.0)
     *               0.5 = giro muy cerrado, 1.0 = retroceso casi recto
     *   girarIzquierda: true = retrocede girando a la izquierda
     *                   false = retrocede girando a la derecha
     *
     * Ejemplo:
     *   retrocederConGiro(100, 0.6, true)
     *   → Retrocede girando a la izquierda
     *   → Rueda izquierda: 60 (60% de 100), Rueda derecha: 100
     ***************************************************************************/
    void retrocederConGiro(uint8_t velocidad, float factorGiro, bool girarIzquierda) {
        velocidad = limitarVelocidad(velocidad);

        // Calcular velocidades diferenciales para el giro
        uint8_t velExterior = velocidad;
        uint8_t velInterior = (uint8_t)(velocidad * factorGiro);

        if (girarIzquierda) {
            // Retrocede girando a la izquierda: rueda izq más lenta
            setMotorIzquierdo(-velInterior);  // Rueda interior
            setMotorDerecho(-velExterior);     // Rueda exterior
        } else {
            // Retrocede girando a la derecha: rueda der más lenta
            setMotorIzquierdo(-velExterior);   // Rueda exterior
            setMotorDerecho(-velInterior);     // Rueda interior
        }

        DEBUG_PRINT(DEBUG_MOTORES, "RetrocesoGiro -> ");
        DEBUG_PRINT(DEBUG_MOTORES, girarIzquierda ? "IZQ" : "DER");
        DEBUG_PRINT(DEBUG_MOTORES, " | Interior: ");
        DEBUG_PRINT(DEBUG_MOTORES, velInterior);
        DEBUG_PRINT(DEBUG_MOTORES, " | Exterior: ");
        DEBUG_PRINTLN(DEBUG_MOTORES, velExterior);
    }

    /***************************************************************************
     * Gira a la derecha sobre su eje (giro en el lugar)
     *
     * Motor izquierdo avanza, motor derecho retrocede
     *
     * Parámetros:
     *   velocidad: Velocidad de giro (0 a 255)
     ***************************************************************************/
    void girarDerecha(uint8_t velocidad) {
        velocidad = limitarVelocidad(velocidad);
        setMotorIzquierdo(velocidad);   // Izquierdo adelante
        setMotorDerecho(-velocidad);    // Derecho atrás

        DEBUG_PRINT(DEBUG_MOTORES, "Girar Derecha -> Velocidad: ");
        DEBUG_PRINTLN(DEBUG_MOTORES, velocidad);
    }

    /***************************************************************************
     * Gira a la izquierda sobre su eje (giro en el lugar)
     *
     * Motor derecho avanza, motor izquierdo retrocede
     *
     * Parámetros:
     *   velocidad: Velocidad de giro (0 a 255)
     ***************************************************************************/
    void girarIzquierda(uint8_t velocidad) {
        velocidad = limitarVelocidad(velocidad);
        setMotorDerecho(velocidad);     // Derecho adelante
        setMotorIzquierdo(-velocidad);  // Izquierdo atrás

        DEBUG_PRINT(DEBUG_MOTORES, "Girar Izquierda -> Velocidad: ");
        DEBUG_PRINTLN(DEBUG_MOTORES, velocidad);
    }

    /***************************************************************************
     * Detiene ambos motores aplicando freno activo
     *
     * Freno activo: IN1=LOW, IN2=LOW (cortocircuita el motor)
     * Es más efectivo que simplemente poner PWM=0
     ***************************************************************************/
    void detener() {
        // Freno activo en motor derecho
        digitalWrite(MOTOR_DER_IN1, LOW);
        digitalWrite(MOTOR_DER_IN2, LOW);
        ledcWrite(PWM_CHANNEL_DER, 0);

        // Freno activo en motor izquierdo
        digitalWrite(MOTOR_IZQ_IN3, LOW);
        digitalWrite(MOTOR_IZQ_IN4, LOW);
        ledcWrite(PWM_CHANNEL_IZQ, 0);

        velocidadActualDerecho = 0;
        velocidadActualIzquierdo = 0;

        DEBUG_PRINTLN(DEBUG_MOTORES, "Motores DETENIDOS (freno activo)");
    }

    /***************************************************************************
     * Detiene los motores gradualmente (rampa de desaceleración)
     *
     * Parámetros:
     *   tiempoRampa: Tiempo de desaceleración en milisegundos
     ***************************************************************************/
    void detenerSuave(uint16_t tiempoRampa = 500) {
        uint8_t velInicialDer = velocidadActualDerecho;
        uint8_t velInicialIzq = velocidadActualIzquierdo;

        unsigned long tiempoInicio = millis();
        unsigned long tiempoTranscurrido;

        DEBUG_PRINT(DEBUG_MOTORES, "Deteniendo suavemente en ");
        DEBUG_PRINT(DEBUG_MOTORES, tiempoRampa);
        DEBUG_PRINTLN(DEBUG_MOTORES, " ms");

        do {
            tiempoTranscurrido = millis() - tiempoInicio;
            float factor = 1.0 - ((float)tiempoTranscurrido / tiempoRampa);
            if (factor < 0) factor = 0;

            uint8_t velDer = velInicialDer * factor;
            uint8_t velIzq = velInicialIzq * factor;

            setMotorDerecho(direccionDerecho ? velDer : -velDer);
            setMotorIzquierdo(direccionIzquierdo ? velIzq : -velIzq);

            delay(20);  // Actualizar cada 20ms
        } while (tiempoTranscurrido < tiempoRampa);

        detener();
    }

    /***************************************************************************
     * Obtiene la velocidad actual promedio de ambos motores
     *
     * Retorna: Velocidad promedio (0-255)
     ***************************************************************************/
    uint8_t obtenerVelocidadPromedio() {
        return (velocidadActualDerecho + velocidadActualIzquierdo) / 2;
    }

    /***************************************************************************
     * Obtiene la velocidad actual del motor derecho
     ***************************************************************************/
    uint8_t obtenerVelocidadDerecho() {
        return velocidadActualDerecho;
    }

    /***************************************************************************
     * Obtiene la velocidad actual del motor izquierdo
     ***************************************************************************/
    uint8_t obtenerVelocidadIzquierdo() {
        return velocidadActualIzquierdo;
    }

    /***************************************************************************
     * Verifica si los motores están en movimiento
     ***************************************************************************/
    bool enMovimiento() {
        return (velocidadActualDerecho > 0 || velocidadActualIzquierdo > 0);
    }

    /***************************************************************************
     * Test de motores - útil para diagnóstico
     *
     * Ejecuta una secuencia de prueba:
     * 1. Motor derecho adelante
     * 2. Motor derecho atrás
     * 3. Motor izquierdo adelante
     * 4. Motor izquierdo atrás
     * 5. Ambos adelante
     * 6. Giro derecha
     * 7. Giro izquierda
     ***************************************************************************/
    void testMotores() {
        Serial.println("\n========================================");
        Serial.println("INICIANDO TEST DE MOTORES");
        Serial.println("========================================\n");

        Serial.println("1. Motor Derecho ADELANTE (2 seg)");
        setMotorDerecho(150);
        setMotorIzquierdo(0);
        delay(2000);
        detener();
        delay(500);

        Serial.println("2. Motor Derecho ATRÁS (2 seg)");
        setMotorDerecho(-150);
        setMotorIzquierdo(0);
        delay(2000);
        detener();
        delay(500);

        Serial.println("3. Motor Izquierdo ADELANTE (2 seg)");
        setMotorDerecho(0);
        setMotorIzquierdo(150);
        delay(2000);
        detener();
        delay(500);

        Serial.println("4. Motor Izquierdo ATRÁS (2 seg)");
        setMotorDerecho(0);
        setMotorIzquierdo(-150);
        delay(2000);
        detener();
        delay(500);

        Serial.println("5. Ambos ADELANTE (2 seg)");
        avanzar(150);
        delay(2000);
        detener();
        delay(500);

        Serial.println("6. Giro DERECHA (2 seg)");
        girarDerecha(120);
        delay(2000);
        detener();
        delay(500);

        Serial.println("7. Giro IZQUIERDA (2 seg)");
        girarIzquierda(120);
        delay(2000);
        detener();

        Serial.println("\n========================================");
        Serial.println("TEST DE MOTORES COMPLETADO");
        Serial.println("========================================\n");
    }

    /***************************************************************************
     * Imprime el estado actual de los motores
     ***************************************************************************/
    void imprimirEstado() {
        Serial.println("\n--- ESTADO DE MOTORES ---");
        Serial.print("Motor Derecho:    ");
        Serial.print(direccionDerecho ? "ADELANTE" : "ATRÁS");
        Serial.print(" | PWM: ");
        Serial.println(velocidadActualDerecho);

        Serial.print("Motor Izquierdo:  ");
        Serial.print(direccionIzquierdo ? "ADELANTE" : "ATRÁS");
        Serial.print(" | PWM: ");
        Serial.println(velocidadActualIzquierdo);

        Serial.print("En movimiento:    ");
        Serial.println(enMovimiento() ? "SÍ" : "NO");

        Serial.print("Velocidad promedio: ");
        Serial.println(obtenerVelocidadPromedio());
        Serial.println("-------------------------\n");
    }
};

#endif // MOTORES_H
