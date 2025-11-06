/*******************************************************************************
 * TEST_MOTORES.ino
 *
 * Sketch de prueba para motores con L298N
 * Ubicación: pruebas/test_motores.ino
 *
 * Hardware:
 * - ESP32-S3 WROOM FREENOVE
 * - L298N (sin jumpers ENA/ENB)
 * - 2x Motores DC
 *
 * CONEXIONES (Corregido según comportamiento real del hardware):
 * Motor Derecho FÍSICO:  GPIO12→ENA, GPIO11→IN1, GPIO18→IN2 (L298N OUT1/OUT2)
 * Motor Izquierdo FÍSICO: GPIO13→ENB, GPIO14→IN3, GPIO21→IN4 (L298N OUT3/OUT4)
 *
 * COMANDOS SERIAL:
 * w = Adelante ambos motores
 * s = Atrás ambos motores
 * a = Girar izquierda
 * d = Girar derecha
 * x = Detener
 * + = Aumentar velocidad
 * - = Disminuir velocidad
 * t = Test de aceleración
 * r = Test de rampa suave
 * p = Test motor DERECHO solo
 * o = Test motor IZQUIERDO solo
 * i = Información
 * h = Ayuda
 *
 * CALIBRACIÓN DE MOTORES:
 * [ = Reducir factor motor DERECHO (-0.01)
 * ] = Aumentar factor motor DERECHO (+0.01)
 * { = Reducir factor motor IZQUIERDO (-0.01)
 * } = Aumentar factor motor IZQUIERDO (+0.01)
 * f = Ver factores actuales
 * c = Calibración guiada paso a paso
 *
 * NOTA: Etiquetas corregidas según comportamiento real observado
 *       Motor DERECHO físico → GPIO12/11/18 (OUT1/OUT2)
 *       Motor IZQUIERDO físico → GPIO13/14/21 (OUT3/OUT4)
 ******************************************************************************/

// Configuración de pines - Corregido según comportamiento real
#define MOTOR_DER_ENA   12    // GPIO12 - PWM motor derecho (ENA del L298N)
#define MOTOR_DER_IN1   11    // GPIO11 - Dirección motor derecho (IN1 del L298N)
#define MOTOR_DER_IN2   18    // GPIO18 - Dirección motor derecho (IN2 del L298N)

#define MOTOR_IZQ_ENB   13    // GPIO13 - PWM motor izquierdo (ENB del L298N)
#define MOTOR_IZQ_IN3   14    // GPIO14 - Dirección motor izquierdo (IN3 del L298N)
#define MOTOR_IZQ_IN4   21    // GPIO21 - Dirección motor izquierdo (IN4 del L298N)

// Configuración PWM (API nueva ESP32 v3.0+)
#define PWM_FREQUENCY   5000  // 5 kHz
#define PWM_RESOLUTION  8     // 8 bits (0-255)

// Variables de control
int velocidad_actual = 150;   // Velocidad inicial (0-255, será mapeada)
const int VEL_MIN = 0;        // Velocidad lógica mínima
const int VEL_MAX = 255;      // Velocidad lógica máxima
const int VEL_INCREMENTO = 10;

// Constantes del sistema
const float VOLTAJE_BATERIA = 12.0;  // Voltaje de alimentación
const float CAIDA_L298N = 2.0;       // Caída de voltaje del L298N

// Rango efectivo de PWM (40% - 100% del duty cycle)
const int PWM_MIN_EFECTIVO = 102;    // 40% de 255 = punto de arranque real
const int PWM_MAX_EFECTIVO = 255;    // 100% máximo

// Factores de compensación para equalizar motores
// AJUSTE: Modificar estos valores para compensar diferencias entre motores
float factor_motor_derecho = 1.00;    // Factor para motor derecho (baseline)
float factor_motor_izquierdo = 1.00;  // Factor para motor izquierdo (ajustar si es más lento)
const float INCREMENTO_FACTOR = 0.01; // Incremento para ajuste fino

// Función auxiliar para limitar valores
int limitarPWM(int valor) {
    if (valor < 0) return 0;
    if (valor > 255) return 255;
    return valor;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n\n╔════════════════════════════════════════════════╗");
    Serial.println("║  TEST DE MOTORES L298N - ESP32-S3 WROOM       ║");
    Serial.println("║  v1.3 - Control por Serial                    ║");
    Serial.println("║  Etiquetas corregidas según hardware real     ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");

    // Configurar pines de dirección como OUTPUT
    pinMode(MOTOR_DER_IN1, OUTPUT);
    pinMode(MOTOR_DER_IN2, OUTPUT);
    pinMode(MOTOR_IZQ_IN3, OUTPUT);
    pinMode(MOTOR_IZQ_IN4, OUTPUT);

    // Configurar PWM con la nueva API (ESP32 v3.0+)
    ledcAttach(MOTOR_DER_ENA, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttach(MOTOR_IZQ_ENB, PWM_FREQUENCY, PWM_RESOLUTION);

    // Detener motores inicialmente
    detenerMotores();

    Serial.println("✅ Inicialización completa\n");
    mostrarAyuda();
    mostrarInfo();
}

void loop() {
    if (Serial.available() > 0) {
        char comando = Serial.read();

        switch (comando) {
            case 'w':  // Adelante
            case 'W':
                avanzar(velocidad_actual);
                Serial.print("⬆️  Adelante - ");
                printVelocidadInfo(velocidad_actual);
                Serial.println();
                break;

            case 's':  // Atrás
            case 'S':
                retroceder(velocidad_actual);
                Serial.print("⬇️  Atrás - ");
                printVelocidadInfo(velocidad_actual);
                Serial.println();
                break;

            case 'a':  // Girar izquierda
            case 'A':
                girarIzquierda(velocidad_actual);
                Serial.print("⬅️  Girar izquierda - ");
                printVelocidadInfo(velocidad_actual);
                Serial.println();
                break;

            case 'd':  // Girar derecha
            case 'D':
                girarDerecha(velocidad_actual);
                Serial.print("➡️  Girar derecha - ");
                printVelocidadInfo(velocidad_actual);
                Serial.println();
                break;

            case 'x':  // Detener
            case 'X':
                detenerMotores();
                Serial.println("⏹️  DETENIDO");
                break;

            case '+':  // Aumentar velocidad
            case '=':
                velocidad_actual = min(velocidad_actual + VEL_INCREMENTO, VEL_MAX);
                Serial.print("🔼 Velocidad aumentada → ");
                printVelocidadInfo(velocidad_actual);
                Serial.println();
                break;

            case '-':  // Disminuir velocidad
            case '_':
                velocidad_actual = max(velocidad_actual - VEL_INCREMENTO, VEL_MIN);
                Serial.print("🔽 Velocidad disminuida → ");
                printVelocidadInfo(velocidad_actual);
                Serial.println();
                break;

            case 't':  // Test de aceleración
            case 'T':
                testAceleracion();
                break;

            case 'r':  // Test de rampa suave
            case 'R':
                testRampaSuave();
                break;

            case 'i':  // Información
            case 'I':
                mostrarInfo();
                break;

            case 'h':  // Ayuda
            case 'H':
            case '?':
                mostrarAyuda();
                break;

            case 'p':  // Test individual motor derecho
            case 'P':
                testMotorDerecho();
                break;

            case 'o':  // Test individual motor izquierdo
            case 'O':
                testMotorIzquierdo();
                break;

            // Comandos de calibración
            case '[':  // Reducir factor motor derecho
                factor_motor_derecho -= INCREMENTO_FACTOR;
                if (factor_motor_derecho < 0.5) factor_motor_derecho = 0.5;
                Serial.print("🔧 Factor motor DERECHO: ");
                Serial.println(factor_motor_derecho, 3);
                break;

            case ']':  // Aumentar factor motor derecho
                factor_motor_derecho += INCREMENTO_FACTOR;
                if (factor_motor_derecho > 1.5) factor_motor_derecho = 1.5;
                Serial.print("🔧 Factor motor DERECHO: ");
                Serial.println(factor_motor_derecho, 3);
                break;

            case '{':  // Reducir factor motor izquierdo
                factor_motor_izquierdo -= INCREMENTO_FACTOR;
                if (factor_motor_izquierdo < 0.5) factor_motor_izquierdo = 0.5;
                Serial.print("🔧 Factor motor IZQUIERDO: ");
                Serial.println(factor_motor_izquierdo, 3);
                break;

            case '}':  // Aumentar factor motor izquierdo
                factor_motor_izquierdo += INCREMENTO_FACTOR;
                if (factor_motor_izquierdo > 1.5) factor_motor_izquierdo = 1.5;
                Serial.print("🔧 Factor motor IZQUIERDO: ");
                Serial.println(factor_motor_izquierdo, 3);
                break;

            case 'f':  // Ver factores
            case 'F':
                mostrarFactores();
                break;

            case 'c':  // Calibración guiada
            case 'C':
                calibracionGuiada();
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
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES AUXILIARES
// ═══════════════════════════════════════════════════════════════

// Mapea velocidad lógica (0-255) a PWM efectivo (102-255)
// Esto permite control lineal usando todo el rango, descartando la zona muerta
int mapearVelocidad(int velocidad_logica) {
    if (velocidad_logica == 0) return 0;  // Detenido = 0 PWM real

    // Mapear linealmente: 1-255 → 102-255 (40%-100%)
    // Formula: pwm = PWM_MIN + (velocidad - 1) * (PWM_MAX - PWM_MIN) / (255 - 1)
    long rango_entrada = 255 - 1;  // 254
    long rango_salida = PWM_MAX_EFECTIVO - PWM_MIN_EFECTIVO;  // 153
    long pwm = PWM_MIN_EFECTIVO + ((velocidad_logica - 1) * rango_salida) / rango_entrada;

    return (int)pwm;
}

// Calcula el voltaje efectivo que llega al motor
float calcularVoltajeEfectivo(int pwm) {
    float porcentaje = (float)pwm / 255.0;
    float voltaje_disponible = VOLTAJE_BATERIA - CAIDA_L298N;
    return voltaje_disponible * porcentaje;
}

// Imprime información detallada de velocidad (mostrando tanto lógica como PWM real)
void printVelocidadInfo(int velocidad_logica) {
    int pwm_real = mapearVelocidad(velocidad_logica);

    Serial.print("VEL:");
    Serial.print(velocidad_logica);
    Serial.print(" → PWM:");
    Serial.print(pwm_real);
    Serial.print(" (");
    Serial.print((pwm_real * 100) / 255);
    Serial.print("% = ");
    Serial.print(calcularVoltajeEfectivo(pwm_real), 1);
    Serial.print("V)");
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES DE CONTROL DE MOTORES
// ═══════════════════════════════════════════════════════════════

void avanzar(int velocidad) {
    int pwm = mapearVelocidad(velocidad);  // Mapear a rango efectivo

    // Aplicar factores de compensación
    int pwm_derecho = limitarPWM((int)(pwm * factor_motor_derecho));
    int pwm_izquierdo = limitarPWM((int)(pwm * factor_motor_izquierdo));

    // Motor derecho adelante
    digitalWrite(MOTOR_DER_IN1, HIGH);
    digitalWrite(MOTOR_DER_IN2, LOW);
    ledcWrite(MOTOR_DER_ENA, pwm_derecho);

    // Motor izquierdo adelante
    digitalWrite(MOTOR_IZQ_IN3, HIGH);
    digitalWrite(MOTOR_IZQ_IN4, LOW);
    ledcWrite(MOTOR_IZQ_ENB, pwm_izquierdo);
}

void retroceder(int velocidad) {
    int pwm = mapearVelocidad(velocidad);  // Mapear a rango efectivo

    // Aplicar factores de compensación
    int pwm_derecho = limitarPWM((int)(pwm * factor_motor_derecho));
    int pwm_izquierdo = limitarPWM((int)(pwm * factor_motor_izquierdo));

    // Motor derecho atrás
    digitalWrite(MOTOR_DER_IN1, LOW);
    digitalWrite(MOTOR_DER_IN2, HIGH);
    ledcWrite(MOTOR_DER_ENA, pwm_derecho);

    // Motor izquierdo atrás
    digitalWrite(MOTOR_IZQ_IN3, LOW);
    digitalWrite(MOTOR_IZQ_IN4, HIGH);
    ledcWrite(MOTOR_IZQ_ENB, pwm_izquierdo);
}

void girarIzquierda(int velocidad) {
    int pwm = mapearVelocidad(velocidad);  // Mapear a rango efectivo

    // Aplicar factores de compensación
    int pwm_derecho = limitarPWM((int)(pwm * factor_motor_derecho));
    int pwm_izquierdo = limitarPWM((int)((pwm / 3) * factor_motor_izquierdo));

    // Motor derecho adelante (más rápido)
    digitalWrite(MOTOR_DER_IN1, HIGH);
    digitalWrite(MOTOR_DER_IN2, LOW);
    ledcWrite(MOTOR_DER_ENA, pwm_derecho);

    // Motor izquierdo detenido o lento
    digitalWrite(MOTOR_IZQ_IN3, HIGH);
    digitalWrite(MOTOR_IZQ_IN4, LOW);
    ledcWrite(MOTOR_IZQ_ENB, pwm_izquierdo);
}

void girarDerecha(int velocidad) {
    int pwm = mapearVelocidad(velocidad);  // Mapear a rango efectivo

    // Aplicar factores de compensación
    int pwm_derecho = limitarPWM((int)((pwm / 3) * factor_motor_derecho));
    int pwm_izquierdo = limitarPWM((int)(pwm * factor_motor_izquierdo));

    // Motor derecho detenido o lento
    digitalWrite(MOTOR_DER_IN1, HIGH);
    digitalWrite(MOTOR_DER_IN2, LOW);
    ledcWrite(MOTOR_DER_ENA, pwm_derecho);

    // Motor izquierdo adelante (más rápido)
    digitalWrite(MOTOR_IZQ_IN3, HIGH);
    digitalWrite(MOTOR_IZQ_IN4, LOW);
    ledcWrite(MOTOR_IZQ_ENB, pwm_izquierdo);
}

void detenerMotores() {
    // Freno activo (ambos pines LOW)
    digitalWrite(MOTOR_DER_IN1, LOW);
    digitalWrite(MOTOR_DER_IN2, LOW);
    ledcWrite(MOTOR_DER_ENA, 0);

    digitalWrite(MOTOR_IZQ_IN3, LOW);
    digitalWrite(MOTOR_IZQ_IN4, LOW);
    ledcWrite(MOTOR_IZQ_ENB, 0);
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES DE TEST
// ═══════════════════════════════════════════════════════════════

void testAceleracion() {
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  TEST DE ACELERACIÓN PROGRESIVA                ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
    Serial.println("Acelerando de 0 a 255 (mapeado a 40%-100% PWM)...");
    Serial.println("Pasos de 25 en velocidad lógica\n");

    for (int vel = 0; vel <= 255; vel += 25) {
        if (vel == 0) {
            Serial.println("Velocidad: VEL:0 → PWM:0 (DETENIDO)");
            detenerMotores();
        } else {
            Serial.print("Velocidad: ");
            printVelocidadInfo(vel);
            Serial.print(" ");

            // Barra de progreso
            Serial.print("[");
            int barras = (vel * 20) / 255;
            for (int i = 0; i < barras; i++) Serial.print("█");
            for (int i = barras; i < 20; i++) Serial.print("░");
            Serial.println("]");

            avanzar(vel);
        }
        delay(1000);
    }

    Serial.println("\n✅ Test completado. Deteniendo motores...\n");
    detenerMotores();
    delay(500);
}

void testRampaSuave() {
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  TEST DE RAMPA SUAVE (0→MAX→0)                ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");

    // Aceleración suave
    Serial.println("⬆️  Acelerando desde 1 hasta 255...");
    for (int vel = 1; vel <= 255; vel += 5) {
        avanzar(vel);

        if (vel % 25 == 0 || vel == 1) {
            Serial.print("  ");
            printVelocidadInfo(vel);
            Serial.println();
        }

        delay(50);
    }

    Serial.println("⏸️  Velocidad máxima (manteniendo 2 seg)...");
    delay(2000);

    // Desaceleración suave
    Serial.println("⬇️  Desacelerando hasta 1...");
    for (int vel = 255; vel >= 1; vel -= 5) {
        avanzar(vel);

        if (vel % 25 == 0 || vel == 1) {
            Serial.print("  ");
            printVelocidadInfo(vel);
            Serial.println();
        }

        delay(50);
    }

    Serial.println("\n✅ Test completado. Motores detenidos.\n");
    detenerMotores();
}

void testMotorDerecho() {
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  TEST MOTOR DERECHO                            ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");

    Serial.println("Adelante...");
    digitalWrite(MOTOR_DER_IN1, HIGH);
    digitalWrite(MOTOR_DER_IN2, LOW);
    ledcWrite(MOTOR_DER_ENA, 180);

    // Motor izquierdo detenido
    digitalWrite(MOTOR_IZQ_IN3, LOW);
    digitalWrite(MOTOR_IZQ_IN4, LOW);
    ledcWrite(MOTOR_IZQ_ENB, 0);

    delay(2000);

    Serial.println("Atrás...");
    digitalWrite(MOTOR_DER_IN1, LOW);
    digitalWrite(MOTOR_DER_IN2, HIGH);
    ledcWrite(MOTOR_DER_ENA, 180);

    delay(2000);

    Serial.println("✅ Test completado\n");
    detenerMotores();
}

void testMotorIzquierdo() {
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  TEST MOTOR IZQUIERDO                          ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");

    Serial.println("Adelante...");
    digitalWrite(MOTOR_IZQ_IN3, HIGH);
    digitalWrite(MOTOR_IZQ_IN4, LOW);
    ledcWrite(MOTOR_IZQ_ENB, 180);

    // Motor derecho detenido
    digitalWrite(MOTOR_DER_IN1, LOW);
    digitalWrite(MOTOR_DER_IN2, LOW);
    ledcWrite(MOTOR_DER_ENA, 0);

    delay(2000);

    Serial.println("Atrás...");
    digitalWrite(MOTOR_IZQ_IN3, LOW);
    digitalWrite(MOTOR_IZQ_IN4, HIGH);
    ledcWrite(MOTOR_IZQ_ENB, 180);

    delay(2000);

    Serial.println("✅ Test completado\n");
    detenerMotores();
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES DE CALIBRACIÓN DE MOTORES
// ═══════════════════════════════════════════════════════════════

void mostrarFactores() {
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  FACTORES DE COMPENSACIÓN ACTUALES             ║");
    Serial.println("╚════════════════════════════════════════════════╝");

    Serial.print("Motor DERECHO:    ");
    Serial.println(factor_motor_derecho, 3);

    Serial.print("Motor IZQUIERDO:  ");
    Serial.println(factor_motor_izquierdo, 3);

    Serial.println("\n💡 Si el robot se desvía:");
    Serial.println("   • Izquierda → Motor izq más lento → Aumentar su factor con }");
    Serial.println("   • Derecha   → Motor der más lento → Aumentar su factor con ]\n");
}

void calibracionGuiada() {
    detenerMotores();
    Serial.println("\n╔════════════════════════════════════════════════╗");
    Serial.println("║  CALIBRACIÓN GUIADA DE MOTORES                 ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");

    Serial.println("📋 PROCEDIMIENTO:");
    Serial.println("1. Coloca el robot en una superficie plana");
    Serial.println("2. Marca una línea recta de 2-3 metros");
    Serial.println("3. Envía 'w' para avanzar");
    Serial.println("4. Observa hacia dónde se desvía:");
    Serial.println("   • Si va a la IZQUIERDA → Motor IZQ más lento");
    Serial.println("   • Si va a la DERECHA   → Motor DER más lento");
    Serial.println("5. Ajusta con [ ] { } y repite hasta que vaya recto");
    Serial.println("6. Envía 'f' para ver factores finales");
    Serial.println("\n💾 IMPORTANTE: Anota los factores finales para");
    Serial.println("   copiarlos en config.h del proyecto principal\n");

    mostrarFactores();
}

// ═══════════════════════════════════════════════════════════════
//  FUNCIONES DE INFORMACIÓN
// ═══════════════════════════════════════════════════════════════

void mostrarAyuda() {
    Serial.println("╔════════════════════════════════════════════════╗");
    Serial.println("║  COMANDOS DISPONIBLES                          ║");
    Serial.println("╠════════════════════════════════════════════════╣");
    Serial.println("║  w = Adelante                                  ║");
    Serial.println("║  s = Atrás                                     ║");
    Serial.println("║  a = Girar izquierda                           ║");
    Serial.println("║  d = Girar derecha                             ║");
    Serial.println("║  x = Detener                                   ║");
    Serial.println("║  + = Aumentar velocidad (+10)                  ║");
    Serial.println("║  - = Disminuir velocidad (-10)                 ║");
    Serial.println("║  ───────────────────────────────────────────   ║");
    Serial.println("║  t = Test de aceleración (0→255)               ║");
    Serial.println("║  r = Test de rampa suave (0→255→0)             ║");
    Serial.println("║  p = Test motor DERECHO solo                   ║");
    Serial.println("║  o = Test motor IZQUIERDO solo                 ║");
    Serial.println("║  ───────────────────────────────────────────   ║");
    Serial.println("║  CALIBRACIÓN DE MOTORES:                       ║");
    Serial.println("║  [ = Reducir factor motor DERECHO (-0.01)      ║");
    Serial.println("║  ] = Aumentar factor motor DERECHO (+0.01)     ║");
    Serial.println("║  { = Reducir factor motor IZQUIERDO (-0.01)    ║");
    Serial.println("║  } = Aumentar factor motor IZQUIERDO (+0.01)   ║");
    Serial.println("║  f = Ver factores actuales                     ║");
    Serial.println("║  c = Calibración guiada paso a paso            ║");
    Serial.println("║  ───────────────────────────────────────────   ║");
    Serial.println("║  i = Mostrar información                       ║");
    Serial.println("║  h = Mostrar esta ayuda                        ║");
    Serial.println("╚════════════════════════════════════════════════╝\n");
}

void mostrarInfo() {
    Serial.println("╔════════════════════════════════════════════════╗");
    Serial.println("║  INFORMACIÓN DEL SISTEMA                       ║");
    Serial.println("╠════════════════════════════════════════════════╣");

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

    Serial.println("║  ───────────────────────────────────────────   ║");
    Serial.println("║  CONFIGURACIÓN MOTORES                         ║");
    Serial.println("║  ───────────────────────────────────────────   ║");

    Serial.print("║  Velocidad actual: ");
    printVelocidadInfo(velocidad_actual);
    Serial.println();

    Serial.println("║  Rango lógico: 0-255 → Rango PWM: 0, 102-255  ║");
    Serial.print("║  PWM efectivo: 40%-100% (");
    Serial.print(PWM_MIN_EFECTIVO);
    Serial.print("-");
    Serial.print(PWM_MAX_EFECTIVO);
    Serial.println(")              ║");

    Serial.print("║  Voltaje batería: ");
    Serial.print(VOLTAJE_BATERIA, 1);
    Serial.println("V");

    Serial.print("║  Caída L298N: ");
    Serial.print(CAIDA_L298N, 1);
    Serial.print("V → Disponible: ");
    Serial.print(VOLTAJE_BATERIA - CAIDA_L298N, 1);
    Serial.println("V");

    Serial.print("║  Frecuencia PWM: ");
    Serial.print(PWM_FREQUENCY);
    Serial.println(" Hz");

    Serial.print("║  Resolución PWM: ");
    Serial.print(PWM_RESOLUTION);
    Serial.println(" bits");

    Serial.println("║  ───────────────────────────────────────────   ║");
    Serial.println("║  COMPENSACIÓN DE MOTORES                       ║");
    Serial.println("║  ───────────────────────────────────────────   ║");

    Serial.print("║  Factor Motor DERECHO:    ");
    Serial.println(factor_motor_derecho, 3);

    Serial.print("║  Factor Motor IZQUIERDO:  ");
    Serial.println(factor_motor_izquierdo, 3);

    Serial.println("║  ───────────────────────────────────────────   ║");
    Serial.println("║  PINES ESP32-S3                                ║");
    Serial.println("║  ───────────────────────────────────────────   ║");

    Serial.println("║  Motor Derecho FÍSICO → L298N OUT1/OUT2:      ║");
    Serial.print("║    ENA (PWM): GPIO");
    Serial.println(MOTOR_DER_ENA);
    Serial.print("║    IN1:       GPIO");
    Serial.println(MOTOR_DER_IN1);
    Serial.print("║    IN2:       GPIO");
    Serial.println(MOTOR_DER_IN2);

    Serial.println("║  Motor Izquierdo FÍSICO → L298N OUT3/OUT4:    ║");
    Serial.print("║    ENB (PWM): GPIO");
    Serial.println(MOTOR_IZQ_ENB);
    Serial.print("║    IN3:       GPIO");
    Serial.println(MOTOR_IZQ_IN3);
    Serial.print("║    IN4:       GPIO");
    Serial.println(MOTOR_IZQ_IN4);

    Serial.println("╚════════════════════════════════════════════════╝\n");
    Serial.println("💡 NOTAS:");
    Serial.println("   • Motor derecho físico → OUT1/OUT2 del L298N");
    Serial.println("   • Motor izquierdo físico → OUT3/OUT4 del L298N");
    Serial.println("   • Mapeo activo: VEL 0→PWM 0, VEL 1-255→PWM 102-255");
    Serial.println("   • Rango muerto (0-40% PWM) eliminado del control");
    Serial.println("   • Control lineal en todo el rango útil 40%-100%\n");
}
