/*******************************************************************************
 * TEST_PINES_ADC.ino (Versión Horizontal con Etiquetas)
 *
 * Herramienta para probar la lectura ADC en pines específicos del ESP32-S3.
 * Cada valor impreso incluye su propio identificador de GPIO para no perder
 * la referencia al hacer scroll.
 *
 * Ubicación: pruebas/test_pines_adc.ino
 *
 * CÓMO USAR:
 * 1. Edita el array `pines_a_probar` con los pines que quieres verificar.
 * 2. Carga el código en tu ESP32-S3.
 * 3. Abre el Monitor Serie (baud rate: 115200).
 * 4. Cada segundo aparecerá una nueva fila con las lecturas en formato:
 *    GPIO2:4095   GPIO4:4012   GPIO5:3980 ...
 *
 ******************************************************************************/

#include <Arduino.h>

// --- CONFIGURACIÓN DE PINES A PROBAR ---
// Array de navegación del robot seguidor de línea (5 sensores)
// Orden de izquierda a derecha según posición física
const int pines_a_probar[] = {
    6,  // GPIO6 - Izquierda +2 (extremo izquierdo)
    5,  // GPIO5 - Izquierda +1
    4,  // GPIO4 - Centro
    8,  // GPIO8 - Derecha +1
    7   // GPIO7 - Derecha +2 (extremo derecho)
};
const int num_pines = sizeof(pines_a_probar) / sizeof(pines_a_probar[0]);

// Etiquetas descriptivas para cada sensor
const char* etiquetas[] = {"IZQ+2", "IZQ+1", "CENTRO", "DER+1", "DER+2"};


// --- SETUP ---
// Se ejecuta una vez al iniciar.
void setup() {
    // Iniciar comunicación serie a 115200 baudios
    Serial.begin(115200);
    delay(1000); // Pequeña pausa para estabilizar y conectar el monitor

    Serial.println();
    Serial.println("╔═════════════════════════════════════════════════════════╗");
    Serial.println("║   Test Array de Navegación - 5 Sensores               ║");
    Serial.println("║   GPIO: 6-5-4-8-7  (IZQ → CENTRO → DER)               ║");
    Serial.println("╚═════════════════════════════════════════════════════════╝");
    Serial.println();

    // Establecer la resolución del ADC a 12 bits (rango 0-4095)
    analogReadResolution(12);

    Serial.println("📊 Configuración de sensores:");
    Serial.println("   IZQ+2(GPIO6) | IZQ+1(GPIO5) | CENTRO(GPIO4) | DER+1(GPIO8) | DER+2(GPIO7)");
    Serial.println();
    Serial.println("Iniciando lecturas continuas cada 1 segundo...");
    Serial.println("-------------------------------------------------------------------");
}


// --- LOOP ---
// Se ejecuta repetidamente.
void loop() {
    // Itera y lee el valor de cada pin
    for (int i = 0; i < num_pines; i++) {
        int pin_actual = pines_a_probar[i];
        int valor_raw = analogRead(pin_actual);

        // Imprime etiqueta descriptiva, GPIO y valor
        Serial.print(etiquetas[i]);
        Serial.print("(G");
        Serial.print(pin_actual);
        Serial.print("):");

        // Formatear el valor para alineación (4 dígitos)
        if (valor_raw < 1000) Serial.print(" ");
        if (valor_raw < 100) Serial.print(" ");
        if (valor_raw < 10) Serial.print(" ");
        Serial.print(valor_raw);

        Serial.print(" | "); // Separador visual
    }

    // Después de imprimir todos los valores de una lectura, haz un salto de línea.
    Serial.println();

    // Espera 1 segundo antes de la siguiente ronda de lecturas
    delay(1000);
}
