/*******************************************************************************
 * TEST_SENSORES.ino
 *
 * Herramienta completa de calibración y diagnóstico de sensores IR
 * Ubicación: pruebas/test_sensores.ino
 *
 * Hardware:
 * - ESP32-S3 WROOM FREENOVE
 * - Array Lejano: 5x HW-511 individuales (GPIO 1-5) [OPCIONAL]
 * - Array Cercano: 1x TCRT5000 módulo 5-en-1 (GPIO 6-10) [ACTIVO]
 *
 * PINES SENSORES ACTIVOS (Array Cercano):
 *   GPIO6  → Sensor 5 (Extremo Izquierdo)  [IZQ]
 *   GPIO7  → Sensor 6 (Interior Izquierdo) [INT_IZQ]
 *   GPIO8  → Sensor 7 (Centro)             [CENTRO]
 *   GPIO9  → Sensor 8 (Interior Derecho)   [INT_DER]
 *   GPIO10 → Sensor 9 (Extremo Derecho)    [DER]
 *
 * COMANDOS BÁSICOS:
 * l = Leer valores RAW (modo continuo ON/OFF)
 * v = Ver valores detallados (una vez)
 * c = Calibrar sensores (proceso guiado)
 *
 * COMANDOS VISUALES:
 * g = Gráfico de barras horizontal
 * b = Detección binaria en tiempo real (█ = negro, ░ = blanco)
 * p = Calcular posición y error para PID
 * w = Vista en forma de onda (histórico de valores)
 *
 * COMANDOS AJUSTE FÍSICO:
 * a = Análisis de alineación (detecta sensores desalineados)
 * d = Test de distancia (óptima entre sensor y pista)
 * s = Test de sensibilidad (detecta sensores defectuosos)
 * e = Test de estabilidad (ruido y variación)
 *
 * COMANDOS CONFIGURACIÓN:
 * m = Mostrar valores min/max calibrados
 * u = Ajustar umbral de detección manualmente
 * r = Reset calibración
 * i = Información del sistema
 * h = Ayuda
 *
 * Versión: 2.0 - Herramienta de Ajuste Fino
 * Fecha: 2025-01-30
 ******************************************************************************/

#include <Arduino.h>  // Incluir explícitamente para PlatformIO

// ═══════════════════════════════════════════════════════════════
//  CONFIGURACIÓN DE PINES
// ═══════════════════════════════════════════════════════════════

// Array Lejano - 5x HW-511 individuales
#define IR_LEJANO_0     1     // GPIO1  (ADC1_CH0) - Extremo Izquierdo
#define IR_LEJANO_1     2     // GPIO2  (ADC1_CH1) - Interior Izquierdo
#define IR_LEJANO_2     3     // GPIO3  (ADC1_CH2) - Centro
#define IR_LEJANO_3     4     // GPIO4  (ADC1_CH3) - Interior Derecho
#define IR_LEJANO_4     5     // GPIO5  (ADC1_CH4) - Extremo Derecho

// Array Cercano - TCRT5000 módulo 5-en-1
#define IR_CERCANO_5    6     // GPIO6  (ADC1_CH5) - Extremo Izquierdo
#define IR_CERCANO_6    7     // GPIO7  (ADC1_CH6) - Interior Izquierdo
#define IR_CERCANO_7    8     // GPIO8  (ADC1_CH7) - Centro
#define IR_CERCANO_8    9     // GPIO9  (ADC1_CH8) - Interior Derecho
#define IR_CERCANO_9    10    // GPIO10 (ADC1_CH9) - Extremo Derecho

// Configuración ADC
#define ADC_RESOLUTION  12    // 12 bits (0-4095)
#define ADC_SAMPLES     10    // Muestras para promedio

// ═══════════════════════════════════════════════════════════════
//  VARIABLES GLOBALES
// ═══════════════════════════════════════════════════════════════

const int NUM_SENSORES = 10;
int pinesSensores[NUM_SENSORES] = {
    IR_LEJANO_0, IR_LEJANO_1, IR_LEJANO_2, IR_LEJANO_3, IR_LEJANO_4,
    IR_CERCANO_5, IR_CERCANO_6, IR_CERCANO_7, IR_CERCANO_8, IR_CERCANO_9
};

// Valores actuales de sensores
int valoresActuales[NUM_SENSORES];

// Calibración
int valoresMin[NUM_SENSORES];  // Valores mínimos por sensor (varía según módulo)
int valoresMax[NUM_SENSORES];  // Valores máximos por sensor (varía según módulo)
bool calibrado = false;

// Lógica REAL observada en hardware (CALIBRACIÓN ACTUALIZADA):
// HW-511 (0-4):    DESHABILITADO - No se usa en navegación actual
// TCRT5000 (5-9):  BLANCO=alto(~4000), NEGRO=bajo(~2000) [ARRAY ACTIVO]
// Nota: Rango más estrecho que lo esperado (2000-4000 vs 0-4095)

// Pesos para cálculo de posición (error)
int pesos[NUM_SENSORES] = {-4, -2, 0, 2, 4, -4, -2, 0, 2, 4};

// Umbrales para detección binaria (se calculan automáticamente)
int umbrales[NUM_SENSORES];

// Modo de lectura continua
bool lecturaActiva = false;

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  TEST DE SENSORES IR - ESP32-S3 WROOM                     ║");
    Serial.println("║  Calibración y Diagnóstico                                ║");
    Serial.println("║  10 Sensores: 5x HW-511 + 5x TCRT5000                    ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    // Configurar resolución ADC
    analogReadResolution(ADC_RESOLUTION);

    // Configurar pines como entrada analógica
    // Para ADC, NO se debe usar pinMode() - analogRead() configura automáticamente
    // Si usas pinMode(INPUT), puede causar problemas con el ADC del ESP32-S3

    // IMPORTANTE: No configurar los pines ADC con pinMode()
    // El ESP32 configura automáticamente los pines cuando se usa analogRead()

    // DIAGNÓSTICO: Leer valores iniciales para verificar que los pines NO inyectan voltaje
    Serial.println("📊 DIAGNÓSTICO INICIAL - Lecturas ADC sin sensores activos:");
    Serial.println("   (Valores deben estar cerca de 0 o flotando ~2000 si no hay nada conectado)\n");

    for (int i = 0; i < NUM_SENSORES; i++) {
        int valorInicial = analogRead(pinesSensores[i]);
        Serial.print("   GPIO");
        Serial.print(pinesSensores[i]);
        Serial.print(": ");
        Serial.print(valorInicial);

        if (valorInicial < 100) {
            Serial.println(" ADC (pin a GND o sin conexión)");
        } else if (valorInicial > 4000) {
            Serial.println(" ADC (pin cerca de 3.3V)");
        } else {
            Serial.println(" ADC (flotando/lectura normal)");
        }
        delay(10);
    }
    Serial.println();

    // Inicializar valores de calibración
    resetearCalibracion();

    Serial.println("✅ Inicialización completa\n");
    mostrarAyuda();
}

// ═══════════════════════════════════════════════════════════════
//  LOOP PRINCIPAL
// ═══════════════════════════════════════════════════════════════

void loop() {
    // Lectura continua si está activada
    if (lecturaActiva) {
        leerSensores();
        mostrarValoresSimple();
        delay(200);
    }

    // Procesar comandos del usuario
    if (Serial.available() > 0) {
        char comando = Serial.read();
        procesarComando(comando);
    }
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES DE LECTURA
// ═══════════════════════════════════════════════════════════════

// Lee todos los sensores con promedio de muestras
void leerSensores() {
    for (int i = 0; i < NUM_SENSORES; i++) {
        long suma = 0;

        // Tomar múltiples muestras y promediar
        for (int j = 0; j < ADC_SAMPLES; j++) {
            suma += analogRead(pinesSensores[i]);
            delayMicroseconds(100);
        }

        valoresActuales[i] = suma / ADC_SAMPLES;
    }
}

// Lee sensores y normaliza (0-1000)
void leerSensoresNormalizados(int valores[]) {
    leerSensores();

    if (!calibrado) {
        // Sin calibración, copiar valores raw
        for (int i = 0; i < NUM_SENSORES; i++) {
            valores[i] = valoresActuales[i];
        }
        return;
    }

    // Normalizar según el tipo de sensor
    // Salida unificada: 0=Negro, 1000=Blanco (independiente del hardware)
    for (int i = 0; i < NUM_SENSORES; i++) {
        int rango = valoresMax[i] - valoresMin[i];

        if (rango == 0) {
            valores[i] = 0;
        } else {
            if (i < 5) {
                // HW-511 (sensores 0-4): Lógica INVERTIDA
                // valoresMin = BLANCO (bajo ~100)
                // valoresMax = NEGRO (alto ~4095)
                // Mapeo: valor bajo (blanco) → salida alta (1000)
                //        valor alto (negro) → salida baja (0)
                long valor = (long)(valoresMax[i] - valoresActuales[i]) * 1000L / rango;
                valores[i] = constrain(valor, 0, 1000);
            } else {
                // TCRT5000 (sensores 5-9): Lógica NATURAL
                // valoresMin = NEGRO (bajo ~650)
                // valoresMax = BLANCO (alto ~4050)
                // Mapeo: valor bajo (negro) → salida baja (0)
                //        valor alto (blanco) → salida alta (1000)
                long valor = (long)(valoresActuales[i] - valoresMin[i]) * 1000L / rango;
                valores[i] = constrain(valor, 0, 1000);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES DE CALIBRACIÓN
// ═══════════════════════════════════════════════════════════════

void calibrarSensores() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  PROCESO DE CALIBRACIÓN - CONFIGURACIÓN REAL              ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");
    Serial.println("📌 COMPORTAMIENTO REAL DE TUS SENSORES:");
    Serial.println("   HW-511 (S0-S4) - [DESHABILITADO]");
    Serial.println("   • No se usa en navegación actual");
    Serial.println();
    Serial.println("   TCRT5000 Módulo (S5-S9) - [ARRAY ACTIVO]:");
    Serial.println("   • BLANCO = Valores ALTOS (~4000)");
    Serial.println("   • NEGRO  = Valores BAJOS (~2000)\n");
    Serial.println("⚠️  NOTA: Rango más estrecho que el teórico (2000-4000 vs 0-4095)\n");

    // FASE 1: Calibrar sobre NEGRO
    Serial.println("📋 FASE 1: Calibración sobre LÍNEA NEGRA");
    Serial.println("   → Coloca el robot sobre la LÍNEA NEGRA");
    Serial.println("   → Asegúrate que todos los sensores estén sobre negro");
    Serial.println("   → Presiona ENTER para continuar...");

    esperarEnter();

    Serial.println("   ⏳ Calibrando NEGRO...");

    // Inicializar con primer valor
    leerSensores();
    for (int i = 0; i < NUM_SENSORES; i++) {
        valoresMin[i] = valoresActuales[i];
        valoresMax[i] = valoresActuales[i];
    }

    // Tomar 50 muestras sobre NEGRO
    for (int muestra = 0; muestra < 50; muestra++) {
        leerSensores();

        for (int i = 0; i < NUM_SENSORES; i++) {
            if (i < 5) {
                // HW-511: Negro = valores ALTOS (buscar máximo)
                if (valoresActuales[i] > valoresMax[i]) {
                    valoresMax[i] = valoresActuales[i];
                }
            } else {
                // TCRT5000: Negro = valores BAJOS (buscar mínimo)
                if (valoresActuales[i] < valoresMin[i]) {
                    valoresMin[i] = valoresActuales[i];
                }
            }
        }

        if (muestra % 10 == 0) {
            Serial.print(".");
        }
        delay(20);
    }

    Serial.println(" ✅ Listo!");
    Serial.print("   HW-511 (S0): ");
    Serial.print(valoresMax[0]);
    Serial.print(" | TCRT5000 (S5): ");
    Serial.println(valoresMin[5]);
    Serial.println();

    // FASE 2: Calibrar sobre BLANCO
    Serial.println("📋 FASE 2: Calibración sobre superficie BLANCA");
    Serial.println("   → Coloca el robot sobre la superficie BLANCA");
    Serial.println("   → Mueve lentamente sobre el blanco (barrido completo)");
    Serial.println("   → Presiona ENTER para continuar...");

    esperarEnter();

    Serial.println("   ⏳ Calibrando BLANCO (10 segundos)...");

    unsigned long tiempoInicio = millis();
    int puntos = 0;

    while (millis() - tiempoInicio < 10000) {
        leerSensores();

        for (int i = 0; i < NUM_SENSORES; i++) {
            if (i < 5) {
                // HW-511: Blanco = valores BAJOS (buscar mínimo)
                if (valoresActuales[i] < valoresMin[i]) {
                    valoresMin[i] = valoresActuales[i];
                }
            } else {
                // TCRT5000: Blanco = valores ALTOS (buscar máximo)
                if (valoresActuales[i] > valoresMax[i]) {
                    valoresMax[i] = valoresActuales[i];
                }
            }
        }

        if (millis() - tiempoInicio > puntos * 1000) {
            Serial.print(".");
            puntos++;
        }

        delay(20);
    }

    Serial.println(" ✅ Listo!");
    Serial.print("   HW-511 (S0): ");
    Serial.print(valoresMin[0]);
    Serial.print(" | TCRT5000 (S5): ");
    Serial.println(valoresMax[5]);
    Serial.println();

    // Calcular umbrales (punto medio entre min y max)
    for (int i = 0; i < NUM_SENSORES; i++) {
        umbrales[i] = (valoresMin[i] + valoresMax[i]) / 2;
    }

    calibrado = true;

    Serial.println("✅ CALIBRACIÓN COMPLETADA\n");
    mostrarValoresCalibracion();
}

void resetearCalibracion() {
    for (int i = 0; i < NUM_SENSORES; i++) {
        valoresMin[i] = 4095;
        valoresMax[i] = 0;
        umbrales[i] = 2048;
    }
    calibrado = false;
    Serial.println("🔄 Calibración reseteada\n");
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES DE VISUALIZACIÓN
// ═══════════════════════════════════════════════════════════════

void mostrarValoresSimple() {
    Serial.print("RAW: ");
    for (int i = 0; i < NUM_SENSORES; i++) {
        Serial.print(valoresActuales[i]);
        Serial.print("\t");

        if (i == 4) Serial.print(" | ");  // Separar arrays
    }
    Serial.println();
}

void mostrarValoresCompleto() {
    leerSensores();

    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  VALORES ACTUALES DE SENSORES                              ║");
    Serial.println("╠════════════════════════════════════════════════════════════╣");
    Serial.println("║  Array LEJANO (HW-511):                                    ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝");

    for (int i = 0; i < 5; i++) {
        mostrarSensorDetalle(i);
    }

    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  Array CERCANO (TCRT5000):                                 ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝");

    for (int i = 5; i < 10; i++) {
        mostrarSensorDetalle(i);
    }

    Serial.println();
}

void mostrarSensorDetalle(int indice) {
    char nombre[30];
    sprintf(nombre, "Sensor %d (GPIO%d)", indice, pinesSensores[indice]);

    Serial.print("  ");
    Serial.print(nombre);
    Serial.print(": ");
    Serial.print(valoresActuales[indice]);

    if (calibrado) {
        Serial.print(" (");
        Serial.print(valoresMin[indice]);
        Serial.print("-");
        Serial.print(valoresMax[indice]);
        Serial.print(")");

        // Indicador visual según tipo de sensor
        if (indice < 5) {
            // HW-511: Lógica invertida (valor alto = negro)
            if (valoresActuales[indice] > umbrales[indice]) {
                Serial.print(" ████ NEGRO");
            } else {
                Serial.print(" ░░░░ BLANCO");
            }
        } else {
            // TCRT5000: Lógica natural (valor bajo = negro)
            if (valoresActuales[indice] < umbrales[indice]) {
                Serial.print(" ████ NEGRO");
            } else {
                Serial.print(" ░░░░ BLANCO");
            }
        }
    }

    Serial.println();
}

void mostrarGraficoBarras() {
    leerSensores();

    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  GRÁFICO DE BARRAS - SENSORES                              ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    Serial.println("Array LEJANO:");
    for (int i = 0; i < 5; i++) {
        dibujarBarra(i);
    }

    Serial.println("\nArray CERCANO:");
    for (int i = 5; i < 10; i++) {
        dibujarBarra(i);
    }

    Serial.println();
}

void dibujarBarra(int indice) {
    Serial.print("S");
    Serial.print(indice);
    Serial.print(": ");

    int valor = valoresActuales[indice];
    int barras = map(valor, 0, 4095, 0, 50);

    for (int i = 0; i < barras; i++) {
        Serial.print("█");
    }

    Serial.print(" ");
    Serial.print(valor);

    if (calibrado && valor > umbrales[indice]) {
        Serial.print(" [BLANCO]");
    } else if (calibrado) {
        Serial.print(" [NEGRO]");
    }

    Serial.println();
}

void mostrarValoresCalibracion() {
    Serial.println("╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  VALORES DE CALIBRACIÓN                                    ║");
    Serial.println("╠════════════════════════════════════════════════════════════╣");

    // Array HW-511
    Serial.println("║  Array HW-511 (BLANCO=bajo, NEGRO=alto):                  ║");
    Serial.println("║  Sensor  │  MIN (Blanco)  │  MAX (Negro)  │  Umbral       ║");
    Serial.println("╠══════════╪════════════════╪═══════════════╪═══════════════╣");

    for (int i = 0; i < 5; i++) {
        char linea[70];
        sprintf(linea, "║  S%-2d     │     %-5d      │     %-5d     │    %-5d      ║",
                i, valoresMin[i], valoresMax[i], umbrales[i]);
        Serial.println(linea);

        // Advertencia para sensor potencialmente defectuoso
        if (i == 2 && valoresMax[i] < 2000) {
            Serial.println("║        ⚠️  ADVERTENCIA: Sensor S2 no alcanza valores normales ║");
        }
    }

    Serial.println("╠════════════════════════════════════════════════════════════╣");
    Serial.println("║  Array TCRT5000 (BLANCO=alto, NEGRO=bajo):                ║");
    Serial.println("║  Sensor  │  MIN (Negro)   │  MAX (Blanco) │  Umbral       ║");
    Serial.println("╠══════════╪════════════════╪═══════════════╪═══════════════╣");

    for (int i = 5; i < NUM_SENSORES; i++) {
        char linea[70];
        sprintf(linea, "║  S%-2d     │     %-5d      │     %-5d     │    %-5d      ║",
                i, valoresMin[i], valoresMax[i], umbrales[i]);
        Serial.println(linea);
    }

    Serial.println("╚════════════════════════════════════════════════════════════╝\n");
    Serial.println("💡 LÓGICA OBSERVADA EN TU HARDWARE (VALORES REALES):");
    Serial.println("   • HW-511:   [DESHABILITADO - No se usa]");
    Serial.println("   • TCRT5000: MIN=Negro(~2000),  MAX=Blanco(~4000)");
    Serial.println("   • Umbral sugerido: ~3000 (punto medio)");
    Serial.println("   • Salida normalizada unificada: 0=Negro, 1000=Blanco\n");
}

void mostrarDeteccionBinaria() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  DETECCIÓN BINARIA (BLANCO/NEGRO)                          ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    if (!calibrado) {
        Serial.println("⚠️  ERROR: Sensores no calibrados. Usa comando 'c' primero.\n");
        return;
    }

    Serial.println("Presiona 'x' para detener...\n");
    Serial.println("Array LEJANO:   Array CERCANO:");

    while (true) {
        leerSensores();

        Serial.print("\r");  // Volver al inicio de la línea

        // Array lejano HW-511 (lógica invertida: valor alto = negro)
        for (int i = 0; i < 5; i++) {
            if (valoresActuales[i] > umbrales[i]) {
                Serial.print("█");  // Negro (valor alto)
            } else {
                Serial.print("░");  // Blanco (valor bajo)
            }
        }

        Serial.print("   ");

        // Array cercano TCRT5000 (lógica natural: valor bajo = negro)
        for (int i = 5; i < 10; i++) {
            if (valoresActuales[i] < umbrales[i]) {
                Serial.print("█");  // Negro (valor bajo)
            } else {
                Serial.print("░");  // Blanco (valor alto)
            }
        }

        Serial.print("   ");

        delay(100);

        // Verificar si el usuario presionó 'x'
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'x' || c == 'X') {
                Serial.println("\n\n✅ Detenido\n");
                break;
            }
        }
    }
}

void calcularPosicion() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  CÁLCULO DE POSICIÓN (ERROR DE LÍNEA)                     ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    if (!calibrado) {
        Serial.println("⚠️  ERROR: Sensores no calibrados. Usa comando 'c' primero.\n");
        return;
    }

    Serial.println("Presiona 'x' para detener...\n");
    Serial.println("Posición │ Error │ Visualización");
    Serial.println("─────────┼───────┼─────────────────────────────────────────");

    while (true) {
        int valoresNorm[NUM_SENSORES];
        leerSensoresNormalizados(valoresNorm);

        // Calcular posición ponderada (usar solo array cercano para el ejemplo)
        long sumaNum = 0;
        long sumaDen = 0;

        for (int i = 5; i < 10; i++) {  // Solo array cercano
            sumaNum += (long)valoresNorm[i] * pesos[i];
            sumaDen += valoresNorm[i];
        }

        int error = 0;
        if (sumaDen > 0) {
            error = sumaNum / sumaDen;
        }

        // Mostrar resultado
        Serial.print("\r");

        // Posición visual
        Serial.print("[");
        int pos = map(error, -4, 4, 0, 40);
        for (int i = 0; i < 40; i++) {
            if (i == pos) {
                Serial.print("█");
            } else if (i == 20) {
                Serial.print("|");
            } else {
                Serial.print("─");
            }
        }
        Serial.print("] ");

        // Error numérico
        char errorStr[10];
        sprintf(errorStr, "%+3d", error);
        Serial.print(errorStr);
        Serial.print("   ");

        delay(100);

        // Verificar si el usuario presionó 'x'
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'x' || c == 'X') {
                Serial.println("\n\n✅ Detenido\n");
                break;
            }
        }
    }
}

void mostrarInfo() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  INFORMACIÓN DEL SISTEMA                                   ║");
    Serial.println("╠════════════════════════════════════════════════════════════╣");

    Serial.print("║  Chip: ");
    Serial.print(ESP.getChipModel());
    Serial.print(" Rev ");
    Serial.println(ESP.getChipRevision());

    Serial.print("║  CPU: ");
    Serial.print(ESP.getCpuFreqMHz());
    Serial.println(" MHz");

    Serial.print("║  RAM libre: ");
    Serial.print(ESP.getFreeHeap() / 1024);
    Serial.println(" KB");

    Serial.println("║  ─────────────────────────────────────────────────────────  ║");
    Serial.println("║  CONFIGURACIÓN SENSORES                                    ║");
    Serial.println("║  ─────────────────────────────────────────────────────────  ║");

    Serial.print("║  Cantidad: ");
    Serial.println(NUM_SENSORES);

    Serial.print("║  Resolución ADC: ");
    Serial.print(ADC_RESOLUTION);
    Serial.print(" bits (0-");
    Serial.print((1 << ADC_RESOLUTION) - 1);
    Serial.println(")");

    Serial.print("║  Muestras por lectura: ");
    Serial.println(ADC_SAMPLES);

    Serial.print("║  Estado calibración: ");
    Serial.println(calibrado ? "✅ CALIBRADO" : "⚠️  NO CALIBRADO");

    Serial.println("╚════════════════════════════════════════════════════════════╝\n");
}

void mostrarAyuda() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  COMANDOS DISPONIBLES - HERRAMIENTA DE DIAGNÓSTICO        ║");
    Serial.println("╠════════════════════════════════════════════════════════════╣");
    Serial.println("║  📊 COMANDOS BÁSICOS                                       ║");
    Serial.println("║  ─────────────────────────────────────────────────────────  ║");
    Serial.println("║  l = Leer valores RAW (modo continuo ON/OFF)              ║");
    Serial.println("║  v = Ver valores detallados (una vez)                     ║");
    Serial.println("║  c = Calibrar sensores (proceso guiado)                   ║");
    Serial.println("║                                                            ║");
    Serial.println("║  📈 COMANDOS VISUALES                                      ║");
    Serial.println("║  ─────────────────────────────────────────────────────────  ║");
    Serial.println("║  g = Gráfico de barras horizontal                         ║");
    Serial.println("║  b = Detección binaria (█ negro / ░ blanco)               ║");
    Serial.println("║  p = Posición y error para PID                            ║");
    Serial.println("║  w = Vista en forma de onda (histórico)                   ║");
    Serial.println("║                                                            ║");
    Serial.println("║  🔧 COMANDOS AJUSTE FÍSICO                                 ║");
    Serial.println("║  ─────────────────────────────────────────────────────────  ║");
    Serial.println("║  a = Análisis de alineación (detecta desalineados)        ║");
    Serial.println("║  d = Test de distancia óptima (2-5mm)                     ║");
    Serial.println("║  s = Test de sensibilidad (detecta defectuosos)           ║");
    Serial.println("║  e = Test de estabilidad (ruido/variación)                ║");
    Serial.println("║                                                            ║");
    Serial.println("║  ⚙️  COMANDOS CONFIGURACIÓN                                ║");
    Serial.println("║  ─────────────────────────────────────────────────────────  ║");
    Serial.println("║  m = Mostrar valores min/max calibrados                   ║");
    Serial.println("║  u = Ajustar umbral manualmente                           ║");
    Serial.println("║  r = Reset calibración                                    ║");
    Serial.println("║  i = Información del sistema                              ║");
    Serial.println("║  h = Mostrar esta ayuda                                   ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");
    Serial.println("💡 TIP: Usa 'x' para salir de modos interactivos\n");
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES AUXILIARES
// ═══════════════════════════════════════════════════════════════

void procesarComando(char comando) {
    switch (comando) {
        case 'l':
        case 'L':
            lecturaActiva = !lecturaActiva;
            if (lecturaActiva) {
                Serial.println("📊 Lectura continua ACTIVADA (presiona 'l' para detener)\n");
            } else {
                Serial.println("⏹️  Lectura continua DESACTIVADA\n");
            }
            break;

        case 'v':
        case 'V':
            mostrarValoresCompleto();
            break;

        case 'c':
        case 'C':
            calibrarSensores();
            break;

        case 'm':
        case 'M':
            if (calibrado) {
                mostrarValoresCalibracion();
            } else {
                Serial.println("⚠️  Sensores no calibrados. Usa comando 'c' primero.\n");
            }
            break;

        case 'g':
        case 'G':
            mostrarGraficoBarras();
            break;

        case 'b':
        case 'B':
            mostrarDeteccionBinaria();
            break;

        case 'p':
        case 'P':
            calcularPosicion();
            break;

        case 'w':
        case 'W':
            vistaOnda();
            break;

        case 'a':
        case 'A':
            analizarAlineacion();
            break;

        case 'd':
        case 'D':
            testDistancia();
            break;

        case 's':
        case 'S':
            testSensibilidad();
            break;

        case 'e':
        case 'E':
            testEstabilidad();
            break;

        case 'u':
        case 'U':
            ajustarUmbral();
            break;

        case 'r':
        case 'R':
            resetearCalibracion();
            break;

        case 'i':
        case 'I':
            mostrarInfo();
            break;

        case 'h':
        case 'H':
        case '?':
            mostrarAyuda();
            break;

        case '\n':
        case '\r':
            // Ignorar saltos de línea
            break;

        default:
            Serial.print("❌ Comando desconocido: '");
            Serial.print(comando);
            Serial.println("' - Presiona 'h' para ayuda");
            break;
    }
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES DE DIAGNÓSTICO AVANZADO
// ═══════════════════════════════════════════════════════════════

void vistaOnda() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  VISTA EN FORMA DE ONDA (Histórico 50 muestras)           ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");
    Serial.println("Presiona 'x' para detener...\n");

    // Buffer circular para histórico
    const int HISTORICO = 50;
    int buffer[5][HISTORICO];
    int indice = 0;

    // Inicializar buffer
    for (int s = 0; s < 5; s++) {
        for (int h = 0; h < HISTORICO; h++) {
            buffer[s][h] = 2000;  // Valor medio
        }
    }

    while (true) {
        leerSensores();

        // Guardar en buffer (solo array cercano: sensores 5-9)
        for (int s = 0; s < 5; s++) {
            buffer[s][indice] = valoresActuales[s + 5];
        }

        indice = (indice + 1) % HISTORICO;

        // Limpiar pantalla (mover cursor arriba)
        Serial.print("\033[2J\033[H");

        // Dibujar onda para cada sensor
        for (int s = 0; s < 5; s++) {
            Serial.print("S");
            Serial.print(s + 5);
            Serial.print(" │");

            for (int h = 0; h < HISTORICO; h++) {
                int idx = (indice + h) % HISTORICO;
                int valor = buffer[s][idx];

                // Escala: 0-4095 → 0-10
                int altura = map(valor, 0, 4095, 0, 10);

                if (altura <= 3) Serial.print("▁");
                else if (altura <= 4) Serial.print("▂");
                else if (altura <= 5) Serial.print("▃");
                else if (altura <= 6) Serial.print("▄");
                else if (altura <= 7) Serial.print("▅");
                else if (altura <= 8) Serial.print("▆");
                else if (altura <= 9) Serial.print("▇");
                else Serial.print("█");
            }

            Serial.print("│ ");
            Serial.println(valoresActuales[s + 5]);
        }

        Serial.println();

        delay(100);

        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'x' || c == 'X') {
                Serial.println("\n✅ Detenido\n");
                break;
            }
        }
    }
}

void analizarAlineacion() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  ANÁLISIS DE ALINEACIÓN DE SENSORES                       ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    if (!calibrado) {
        Serial.println("⚠️  Primero calibra los sensores (comando 'c')\n");
        return;
    }

    Serial.println("📋 Coloca el robot con la línea CENTRADA bajo el sensor 7");
    Serial.println("   Presiona ENTER para medir...");
    esperarEnter();

    // Tomar 20 muestras
    int sumaSensores[5] = {0, 0, 0, 0, 0};

    for (int i = 0; i < 20; i++) {
        leerSensores();
        for (int s = 0; s < 5; s++) {
            sumaSensores[s] += valoresActuales[s + 5];
        }
        delay(50);
    }

    // Promediar
    int promSensores[5];
    for (int s = 0; s < 5; s++) {
        promSensores[s] = sumaSensores[s] / 20;
    }

    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  RESULTADOS DE ALINEACIÓN                                  ║");
    Serial.println("╠════════════════════════════════════════════════════════════╣");

    // El sensor central (S7) debería tener el valor más bajo (más negro)
    int sensorMasBajo = 0;
    int valorMasBajo = promSensores[0];

    for (int s = 1; s < 5; s++) {
        if (promSensores[s] < valorMasBajo) {
            valorMasBajo = promSensores[s];
            sensorMasBajo = s;
        }
    }

    for (int s = 0; s < 5; s++) {
        Serial.print("║  S");
        Serial.print(s + 5);
        Serial.print(": ");
        Serial.print(promSensores[s]);

        if (s == 2) {  // S7 = Centro
            Serial.print(" [CENTRO ESPERADO]");
            if (sensorMasBajo == 2) {
                Serial.println(" ✅ OK");
            } else {
                Serial.println(" ⚠️  DESALINEADO!");
            }
        } else if (s == sensorMasBajo) {
            Serial.println(" ⚠️  DETECTA MÁS LÍNEA (debería ser S7)");
        } else {
            int diferencia = promSensores[s] - valorMasBajo;
            Serial.print(" (diff: +");
            Serial.print(diferencia);
            Serial.println(")");
        }
    }

    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    if (sensorMasBajo != 2) {
        Serial.println("⚠️  DIAGNÓSTICO: Robot descentrado o sensores desalineados");
        Serial.println("   SOLUCIÓN:");
        Serial.println("   1. Verifica montaje físico del array");
        Serial.println("   2. Ajusta distancia de todos los sensores por igual");
        Serial.println("   3. Asegúrate que el array esté perpendicular a la línea\n");
    } else {
        Serial.println("✅ Alineación correcta!\n");
    }
}

void testDistancia() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  TEST DE DISTANCIA ÓPTIMA (Sensor a Pista)                ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    Serial.println("📋 Instrucciones:");
    Serial.println("   1. Coloca el robot sobre la línea negra");
    Serial.println("   2. Comienza con sensores MUY CERCA (1mm)");
    Serial.println("   3. Levanta lentamente mientras observas valores");
    Serial.println("   4. Presiona 'x' cuando encuentres distancia óptima\n");
    Serial.println("🎯 Objetivo: Máximo contraste (negro más bajo posible)\n");
    Serial.println("Presiona ENTER para iniciar...");
    esperarEnter();

    Serial.println("\n┌──────┬────────────────────────────────────────────────┐");
    Serial.println("│ Dist │  S5    S6    S7    S8    S9   │ Contraste     │");
    Serial.println("├──────┼────────────────────────────────────────────────┤");

    int mejorContraste = 0;
    String mejorMomento = "";

    while (true) {
        leerSensores();

        // Calcular contraste promedio (diferencia blanco-negro)
        int contrasteTotal = 0;
        for (int s = 5; s < 10; s++) {
            int contraste = valoresMax[s] - valoresMin[s];
            contrasteTotal += contraste;
        }
        int contrastePromedio = contrasteTotal / 5;

        if (contrastePromedio > mejorContraste) {
            mejorContraste = contrastePromedio;
            mejorMomento = "AHORA";
        } else {
            mejorMomento = "";
        }

        Serial.print("\r│ Ahora│ ");
        for (int s = 5; s < 10; s++) {
            Serial.print(valoresActuales[s]);
            Serial.print(" ");
        }
        Serial.print(" │ ");
        Serial.print(contrastePromedio);
        Serial.print(" ");
        Serial.print(mejorMomento);
        Serial.print("   ");

        delay(100);

        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == 'x' || c == 'X') {
                Serial.println("\n└──────┴────────────────────────────────────────────────┘\n");
                Serial.print("✅ Mejor contraste observado: ");
                Serial.println(mejorContraste);
                Serial.println("\n📏 RECOMENDACIONES TCRT5000:");
                Serial.println("   • Distancia óptima: 2-4mm");
                Serial.println("   • Contraste mínimo: 2500 ADC");
                Serial.println("   • Si contraste < 2000: Acerca más los sensores\n");
                break;
            }
        }
    }
}

void testSensibilidad() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  TEST DE SENSIBILIDAD (Detecta Sensores Defectuosos)      ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    Serial.println("📋 Este test detecta:");
    Serial.println("   • Sensores con poca respuesta (sucios/defectuosos)");
    Serial.println("   • Sensores saturados (muy cerca o LED quemado)");
    Serial.println("   • Sensores con rango dinámico insuficiente\n");

    if (!calibrado) {
        Serial.println("⚠️  Primero calibra los sensores (comando 'c')\n");
        return;
    }

    Serial.println("╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  Sensor │  MIN   │  MAX   │  Rango │ Estado             ║");
    Serial.println("╠═════════╪════════╪════════╪════════╪════════════════════╣");

    bool todoOK = true;

    for (int i = 0; i < NUM_SENSORES; i++) {
        int rango = valoresMax[i] - valoresMin[i];
        String estado = "";
        String icono = "";

        if (rango < 1000) {
            estado = "⚠️  POCO CONTRASTE";
            icono = "⚠️ ";
            todoOK = false;
        } else if (rango < 2000) {
            estado = "⚠️  CONTRASTE BAJO";
            icono = "⚠️ ";
            todoOK = false;
        } else if (valoresMax[i] > 4000 && valoresMin[i] < 200) {
            estado = "✅ EXCELENTE";
            icono = "✅";
        } else if (valoresMax[i] < 3000) {
            estado = "⚠️  NO LLEGA A BLANCO";
            icono = "⚠️ ";
            todoOK = false;
        } else if (valoresMin[i] > 1500) {
            estado = "⚠️  NO LLEGA A NEGRO";
            icono = "⚠️ ";
            todoOK = false;
        } else {
            estado = "✅ OK";
            icono = "✅";
        }

        char linea[80];
        sprintf(linea, "║   S%-2d   │ %-6d │ %-6d │ %-6d │ %s %-15s ║",
                i, valoresMin[i], valoresMax[i], rango, icono.c_str(), estado.c_str());
        Serial.println(linea);
    }

    Serial.println("╚═════════╧════════╧════════╧════════╧════════════════════╝\n");

    if (todoOK) {
        Serial.println("✅ Todos los sensores funcionan correctamente!\n");
    } else {
        Serial.println("⚠️  PROBLEMAS DETECTADOS:");
        Serial.println("   • Rango < 2000: Limpia sensor o ajusta distancia");
        Serial.println("   • No llega a blanco: Sensor muy lejos o LED débil");
        Serial.println("   • No llega a negro: Sensor muy cerca o saturado\n");
    }
}

void testEstabilidad() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  TEST DE ESTABILIDAD (Ruido y Variación)                  ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    Serial.println("📋 Instrucciones:");
    Serial.println("   1. Coloca el robot sobre la línea NEGRA");
    Serial.println("   2. NO MUEVAS el robot durante el test");
    Serial.println("   3. El test medirá variación en 100 muestras (5 seg)\n");
    Serial.println("Presiona ENTER para iniciar...");
    esperarEnter();

    Serial.println("⏳ Midiendo estabilidad...\n");

    // Tomar 100 muestras
    int minMuestra[10], maxMuestra[10];
    for (int i = 0; i < 10; i++) {
        minMuestra[i] = 4095;
        maxMuestra[i] = 0;
    }

    for (int muestra = 0; muestra < 100; muestra++) {
        leerSensores();

        for (int s = 0; s < NUM_SENSORES; s++) {
            if (valoresActuales[s] < minMuestra[s]) {
                minMuestra[s] = valoresActuales[s];
            }
            if (valoresActuales[s] > maxMuestra[s]) {
                maxMuestra[s] = valoresActuales[s];
            }
        }

        if (muestra % 10 == 0) {
            Serial.print(".");
        }

        delay(50);
    }

    Serial.println(" ✅\n");

    Serial.println("╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  Sensor │ Variación │ Ruido % │ Estado                   ║");
    Serial.println("╠═════════╪═══════════╪═════════╪══════════════════════════╣");

    bool todoOK = true;

    for (int i = 0; i < NUM_SENSORES; i++) {
        int variacion = maxMuestra[i] - minMuestra[i];
        float ruidoPorcentaje = (variacion * 100.0) / 4095.0;
        String estado = "";

        if (variacion < 50) {
            estado = "✅ EXCELENTE (<1%)";
        } else if (variacion < 100) {
            estado = "✅ BUENO (<2%)";
        } else if (variacion < 200) {
            estado = "⚠️  RUIDO MODERADO";
            todoOK = false;
        } else {
            estado = "❌ RUIDO EXCESIVO";
            todoOK = false;
        }

        char linea[80];
        sprintf(linea, "║   S%-2d   │   %-6d  │  %5.2f  │ %-24s ║",
                i, variacion, ruidoPorcentaje, estado.c_str());
        Serial.println(linea);
    }

    Serial.println("╚═════════╧═══════════╧═════════╧══════════════════════════╝\n");

    if (todoOK) {
        Serial.println("✅ Sensores estables, bajo ruido!\n");
    } else {
        Serial.println("⚠️  RUIDO DETECTADO:");
        Serial.println("   CAUSAS POSIBLES:");
        Serial.println("   • Cables sueltos o sin blindar");
        Serial.println("   • Interferencia electromagnética (motores cerca)");
        Serial.println("   • Alimentación inestable (batería baja)");
        Serial.println("   • Luz ambiental variable\n");
        Serial.println("   SOLUCIONES:");
        Serial.println("   • Usa cables cortos y trenzados");
        Serial.println("   • Aleja sensores de motores/WiFi");
        Serial.println("   • Agrega condensadores (100nF) en VCC de sensores");
        Serial.println("   • Aumenta ADC_SAMPLES en código (línea 60)\n");
    }
}

void ajustarUmbral() {
    Serial.println("\n╔════════════════════════════════════════════════════════════╗");
    Serial.println("║  AJUSTE MANUAL DE UMBRAL                                   ║");
    Serial.println("╚════════════════════════════════════════════════════════════╝\n");

    if (!calibrado) {
        Serial.println("⚠️  Primero calibra los sensores (comando 'c')\n");
        return;
    }

    Serial.println("Umbrales actuales:");
    for (int i = 0; i < NUM_SENSORES; i++) {
        Serial.print("  S");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(umbrales[i]);
    }

    Serial.println("\n¿Usar umbrales calculados automáticamente? (s/n)");

    while (!Serial.available()) delay(10);
    char resp = Serial.read();

    if (resp == 's' || resp == 'S') {
        // Recalcular umbrales como punto medio
        for (int i = 0; i < NUM_SENSORES; i++) {
            umbrales[i] = (valoresMin[i] + valoresMax[i]) / 2;
        }
        Serial.println("\n✅ Umbrales recalculados automáticamente\n");
    } else {
        Serial.println("\nIngresa nuevo umbral global (0-4095):");

        while (!Serial.available()) delay(10);
        int nuevoUmbral = Serial.parseInt();

        if (nuevoUmbral >= 0 && nuevoUmbral <= 4095) {
            for (int i = 0; i < NUM_SENSORES; i++) {
                umbrales[i] = nuevoUmbral;
            }
            Serial.print("\n✅ Umbral global ajustado a: ");
            Serial.println(nuevoUmbral);
            Serial.println();
        } else {
            Serial.println("\n❌ Valor inválido\n");
        }
    }

    mostrarValoresCalibracion();
}

void esperarEnter() {
    while (true) {
        if (Serial.available() > 0) {
            char c = Serial.read();
            if (c == '\n' || c == '\r') {
                // Limpiar buffer
                while (Serial.available() > 0) {
                    Serial.read();
                }
                break;
            }
        }
        delay(10);
    }
}
