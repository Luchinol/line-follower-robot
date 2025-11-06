/*******************************************************************************
 * SENSORES.H - Gestión de Sensores IR para un único array
 *
 * Este archivo maneja toda la lógica relacionada con los sensores IR:
 * - Lectura de valores analógicos de N sensores (definido en config.h)
 * - Calibración automática para encontrar mínimos (blanco) y máximos (negro)
 * - Cálculo de la posición ponderada de la línea
 * - Detección de si la línea está visible o no
 *
 * Autor: LUCHIN-OPRESORCL (Refactorizado por Gemini)
 * Fecha: 2025-11-05
 * Versión: 2.0.0
 ******************************************************************************/

#ifndef SENSORES_H
#define SENSORES_H

#include "config.h"

/*******************************************************************************
 * CLASE: SensoresIR
 *
 * Gestiona un array de N sensores IR (3 o 5 según config.h)
 ******************************************************************************/
class SensoresIR {
private:
    // --- Variables de Sensores ---
    uint16_t valoresCrudos[NUM_SENSORES];
    uint16_t valoresNormalizados[NUM_SENSORES];

    // --- Variables de Calibración ---
    uint16_t minCalibracion[NUM_SENSORES];
    uint16_t maxCalibracion[NUM_SENSORES];
    bool calibrado = false;
    unsigned long tiempoInicioCalib = 0;

    // --- Estado de la Línea ---
    int16_t ultimaPosicion = 0;
    bool lineaDetectada = false;

    /***************************************************************************
     * Mapea un valor del rango ADC al rango normalizado (0-1000)
     * usando los valores min/max obtenidos durante la calibración.
     ***************************************************************************/
    uint16_t mapearValor(uint16_t valor, uint16_t min, uint16_t max) {
        if (max <= min) {
            return 0; // Evita división por cero si la calibración fue mala
        }
        uint16_t valor_limitado = constrain(valor, min, max);
        return map(valor_limitado, min, max, 0, 1000);
    }

    /***************************************************************************
     * Calcula la posición ponderada de la línea.
     * Retorna: Posición de -100 a +100 (3 sensores) o -200 a +200 (5 sensores)
     ***************************************************************************/
    int16_t calcularPosicion() {
        long sumaPonderada = 0;
        long sumaTotal = 0;
        bool algunaLectura = false;

        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            // Normalizamos el valor a 0-100 para el cálculo
            int val = valoresNormalizados[i] / 10;
            if (val > 5) { // Umbral para considerar una lectura válida
                sumaPonderada += (long)val * PESOS_SENSORES[i];
                sumaTotal += val;
                algunaLectura = true;
            }
        }

        if (!algunaLectura) {
            lineaDetectada = false;
            // Si no se detecta línea, devolvemos la última posición conocida
            return ultimaPosicion;
        }

        lineaDetectada = true;
        ultimaPosicion = (sumaPonderada * 100) / sumaTotal; // Escalamos para más resolución
        return ultimaPosicion;
    }

public:
    /***************************************************************************
     * Constructor - Inicializa los arrays de calibración
     ***************************************************************************/
    SensoresIR() {
        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            minCalibracion[i] = ADC_MAX_VALUE;
            maxCalibracion[i] = 0;
            valoresCrudos[i] = 0;
            valoresNormalizados[i] = 0;
        }
    }

    /***************************************************************************
     * Inicializa los pines de los sensores como entradas analógicas
     ***************************************************************************/
    void inicializar() {
        DEBUG_PRINTLN(DEBUG_SENSORES, "Inicializando array de sensores...");

        // IMPORTANTE: NO usar pinMode() en pines ADC del ESP32-S3
        // analogRead() configura automáticamente los pines en modo alta impedancia
        // Usar pinMode(INPUT) puede causar problemas de inyección de corriente

        analogReadResolution(ADC_RESOLUTION);
        analogSetAttenuation(ADC_11db); // Rango completo 0-3.3V

        DEBUG_PRINTLN(DEBUG_SENSORES, "Sensores IR inicializados.");
    }

    /***************************************************************************
     * Inicia el proceso de calibración automática
     ***************************************************************************/
    void iniciarCalibracion() {
        Serial.println("\n======================================");
        Serial.println("INICIANDO CALIBRACIÓN DE SENSORES");
        Serial.println("======================================");
        Serial.printf("Mueva el robot sobre blanco y negro durante %d segundos...\n", TIEMPO_CALIBRACION / 1000);

        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            minCalibracion[i] = ADC_MAX_VALUE;
            maxCalibracion[i] = 0;
        }

        calibrado = false;
        tiempoInicioCalib = millis();
    }

    /***************************************************************************
     * Actualiza el proceso de calibración (llamar en cada loop)
     ***************************************************************************/
    bool actualizarCalibracion() {
        if (calibrado) return true;

        if (millis() - tiempoInicioCalib > TIEMPO_CALIBRACION) {
            calibrado = true;
            Serial.println("\n======================================");
            Serial.println("CALIBRACIÓN COMPLETADA");
            Serial.println("======================================");
            imprimirValoresCalibracion();
            return true;
        }

        leer();
        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            if (valoresCrudos[i] < minCalibracion[i]) minCalibracion[i] = valoresCrudos[i];
            if (valoresCrudos[i] > maxCalibracion[i]) maxCalibracion[i] = valoresCrudos[i];
        }

        static unsigned long ultimoReporte = 0;
        if (millis() - ultimoReporte > 1000) {
            ultimoReporte = millis();
            unsigned long restante = (TIEMPO_CALIBRACION - (millis() - tiempoInicioCalib)) / 1000;
            Serial.printf("Calibrando... %lu segundos restantes\n", restante);
        }

        return false;
    }

    /***************************************************************************
     * Lee los valores crudos de todos los sensores IR
     ***************************************************************************/
    void leer() {
        #if NUM_SENSORES >= 1
            valoresCrudos[0] = analogRead(SENSOR_1_PIN);
        #endif
        #if NUM_SENSORES >= 2
            valoresCrudos[1] = analogRead(SENSOR_2_PIN);
        #endif
        #if NUM_SENSORES >= 3
            valoresCrudos[2] = analogRead(SENSOR_3_PIN);
        #endif
        #if NUM_SENSORES >= 4
            valoresCrudos[3] = analogRead(SENSOR_4_PIN);
        #endif
        #if NUM_SENSORES >= 5
            valoresCrudos[4] = analogRead(SENSOR_5_PIN);
        #endif
    }

    /***************************************************************************
     * Procesa las lecturas: normaliza valores y calcula la posición de la línea
     ***************************************************************************/
    int16_t procesar() {
        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            valoresNormalizados[i] = mapearValor(valoresCrudos[i], minCalibracion[i], maxCalibracion[i]);
        }
        return calcularPosicion();
    }

    /***************************************************************************
     * Verifica si la línea está siendo detectada
     ***************************************************************************/
    bool isLineaDetectada() {
        return lineaDetectada;
    }

    /***************************************************************************
     * Imprime los valores de calibración por Serial
     ***************************************************************************/
    void imprimirValoresCalibracion() {
        Serial.println("\nVALORES DE CALIBRACIÓN:");
        Serial.println("Sensor | Min  | Max  | Rango");
        Serial.println("-------|------|------|-------");
        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            Serial.printf("  %d    | %4d | %4d | %4d\n",
                i + 1,
                minCalibracion[i],
                maxCalibracion[i],
                maxCalibracion[i] - minCalibracion[i]
            );
        }
        Serial.println();
    }

    /***************************************************************************
     * Imprime los valores actuales de los sensores
     ***************************************************************************/
    void imprimirValores() {
        Serial.println("\nVALORES ACTUALES DE SENSORES:");
        Serial.print("Crudos:    ");
        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            Serial.printf("%4d ", valoresCrudos[i]);
        }
        Serial.println();

        Serial.print("Normalizados (0-1000): ");
        for (uint8_t i = 0; i < NUM_SENSORES; i++) {
            Serial.printf("%4d ", valoresNormalizados[i]);
        }
        Serial.println();
        Serial.printf("Posición de la línea: %d\n", ultimaPosicion);
        Serial.println();
    }
};

#endif // SENSORES_H