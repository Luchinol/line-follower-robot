# 🤖 Robot Seguidor de Línea - ESP32-S3

Prueba de commit


Sistema de seguimiento de línea autónomo con **ESP32-S3**, **5 sensores IR** y **control PID adaptativo**. Diseñado para máxima precisión y estabilidad en trayectorias complejas.

## 🎯 Hardware Utilizado

| Componente                 | Modelo                            | Cantidad |
| -------------------------- | --------------------------------- | -------- |
| **Microcontrolador** | ESP32-S3 WROOM (FREENOVE)         | 1        |
| **Sensores IR**      | HW-511 (analógicos individuales) | 5        |
| **Puente H**         | L298N                             | 1        |
| **Motores DC**       | Con reductora 1:48                | 2        |
| **Batería**         | LiPo 2S 7.4V o 6xAA               | 1        |

### 📐 Especificaciones de Sensores

- **Tipo**: HW-511 (salida analógica)
- **Valores calibrados**:
  - BLANCO: ~100 ADC (12-bit)
  - NEGRO: ~2000 ADC
- **Resolución espacial**: 5 sensores con pesos -2, -1, 0, +1, +2
- **Rango de error**: -200 a +200

## ⚡ Inicio Rápido

### 1. Conexiones Hardware

```
MOTORES (ESP32-S3 → L298N):
  GPIO 12 → ENA    // PWM Motor Derecho
  GPIO 11 → IN1    // Dirección Motor Derecho
  GPIO 18 → IN2    // Dirección Motor Derecho

  GPIO 13 → ENB    // PWM Motor Izquierdo
  GPIO 14 → IN3    // Dirección Motor Izquierdo
  GPIO 21 → IN4    // Dirección Motor Izquierdo

SENSORES (5x HW-511 → ESP32-S3):
  Array de navegación (izquierda → derecha):

  GPIO 6 → Sensor 1 (IZQ+2, extremo izquierdo)  | Peso: -2
  GPIO 5 → Sensor 2 (IZQ+1)                     | Peso: -1
  GPIO 4 → Sensor 3 (CENTRO)                    | Peso:  0
  GPIO 8 → Sensor 4 (DER+1)                     | Peso: +1
  GPIO 7 → Sensor 5 (DER+2, extremo derecho)    | Peso: +2

ALIMENTACIÓN:
  Batería 7.4V → L298N (+12V)
  L298N 5V OUT → Sensores VCC (todos)
  L298N 5V OUT → ESP32-S3 VIN (si no usas USB)
  GND común → Conectar TODOS los GND juntos
```

> ⚠️ **IMPORTANTE**: Los pines ADC del ESP32-S3 **NO deben** configurarse con `pinMode()`.
> El framework Arduino los configura automáticamente en modo alta impedancia al usar `analogRead()`.

### 2. Programar

```bash
# Usando PlatformIO (Recomendado)
pio run -t upload && pio device monitor

# Usando Arduino IDE
# Placa: "ESP32S3 Dev Module"
# USB CDC On Boot: "Enabled"
```

### 3. Calibrar Sensores

**Este es el paso más crítico para el correcto funcionamiento.**

```
1. Abrir el Monitor Serial (baudrate: 115200)
2. Enviar el comando 'c' para iniciar calibración
3. Durante 8 segundos, mover el robot manualmente asegurando que:
   - TODOS los 5 sensores pasen sobre la LÍNEA NEGRA
   - TODOS los 5 sensores pasen sobre la superficie BLANCA
   - Se recorra varias veces para capturar valores extremos
4. El sistema guarda automáticamente los valores min/max de cada sensor
5. Verifica los valores con el comando 's' (estado)
```

**Valores esperados después de calibración:**

- Sensores sobre BLANCO: ~100-300 ADC
- Sensores sobre NEGRO: ~1800-2200 ADC
- Umbral automático: punto medio entre min/max

### 4. ¡A rodar!

El robot iniciará el seguimiento de línea automáticamente después de la calibración. Usa los comandos seriales para pausar, ajustar o reiniciar.

## 🎮 Comandos Seriales

| Comando              | Descripción                                                                       |
| -------------------- | ---------------------------------------------------------------------------------- |
| `c`                | **Iniciar calibración** de sensores.                                        |
| `s`                | Ver**estado** del sistema (estado, velocidad, PID).                          |
| `r`                | **Reiniciar** el seguimiento de línea.                                      |
| `d`                | Ejecutar un**diagnóstico** de hardware.                                     |
| `p [Kp] [Ki] [Kd]` | Ajustar los parámetros**PID** en tiempo real.                               |
| `v [vel]`          | Cambiar la**velocidad base** del robot (0-255).                              |
| `save`             | **Guardar** la configuración actual de PID y velocidad en la memoria Flash. |
| `h` o `?`        | Mostrar la lista completa de**ayuda**.                                       |
| `0` / `1`        | Atajos para**pausar** y **reanudar**.                                  |

**Ejemplo de ajuste:**
`p 2.5 0.03 0.5` - Ajusta los parámetros PID a los valores por defecto para una recta.
`v 140` - Establece la velocidad base a 140.
`save` - Guarda estos nuevos valores para que se usen la próxima vez que enciendas el robot.

## 🔧 Características Principales

### Control PID Adaptativo con 3 Modos

El sistema ajusta automáticamente los parámetros PID según la dificultad de la trayectoria:

#### **Modo RECTA** (trayectorias rectas)

- **Kp = 1.2** - Respuesta proporcional suave
- **Ki = 0.01** - Corrección integral mínima
- **Kd = 0.8** - Amortiguación para evitar oscilaciones
- **Velocidad**: 120 PWM (conservadora)

#### **Modo CURVA SUAVE** (curvas graduales)

- **Kp = 1.8** - Mayor respuesta proporcional
- **Ki = 0.02** - Integral ligeramente mayor
- **Kd = 1.0** - Mayor amortiguación
- **Velocidad**: 85% de velocidad base

#### **Modo CURVA CERRADA** (curvas muy pronunciadas)

- **Kp = 2.5** - Respuesta agresiva
- **Ki = 0.0** - Sin integral (evita wind-up)
- **Kd = 1.2** - Amortiguación máxima
- **Velocidad**: 60% de velocidad base (90 PWM)

### Algoritmo de Detección de Línea

**5 sensores con pesos espaciales:**

```
[-2]  [-1]  [0]  [+1]  [+2]
 IZQ   IZQ  CEN  DER   DER
 +2    +1        +1    +2
```

**Cálculo de error ponderado:**

```cpp
error = Σ(valor_normalizado[i] × peso[i]) / Σ(valor_normalizado[i])
```

**Rango de error:** -200 (extremo izquierdo) a +200 (extremo derecho)

**Ventajas de 5 sensores:**

- ✅ Mayor resolución espacial (vs 3 sensores)
- ✅ Mejor anticipación en curvas
- ✅ Detección más precisa del centro de línea
- ✅ Permite PID más suave (menos oscilaciones)

### Persistencia de Configuración (NVS)

Gracias al módulo `nvs_config.h`, puedes ajustar los parámetros PID y la velocidad base a través del monitor serial y guardarlos. No se perderán al apagar el robot.

## 🐛 Troubleshooting Rápido

| Problema                           | Solución Sugerida                                                                                                |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| **Oscila mucho en la recta** | El parámetro `Kp` es muy alto. Redúcelo con `p [nuevo_kp] [ki] [kd]` y guarda.                              |
| **Se sale en las curvas**    | La velocidad es muy alta o `Kp` es muy bajo. Prueba bajar la velocidad con `v [nueva_vel]` o subir `Kp`.    |
| **Se mueve erráticamente**  | La calibración falló. Recalibra (`c`) asegurándote de que todos los sensores vean bien el blanco y el negro. |
| **Un motor gira más lento** | Ajusta el `FACTOR_MOTOR_IZQUIERDO` o `FACTOR_MOTOR_DERECHO` en `src/config.h`.                              |
| **No responde a comandos**   | Verifica que el baudrate del monitor serial sea `115200` y que la línea termine en `NL & CR`.                |

## 📂 Estructura del Proyecto

```
CarritoSeguidor/
├── README.md                    # 📖 Esta guía completa
├── platformio.ini               # ⚙️  Configuración de PlatformIO
├── LICENSE                      # 📄 Licencia MIT
├── .gitignore                   # 🚫 Archivos excluidos de Git
│
├── src/                         # 💻 Código fuente principal
│   ├── main.cpp                 # 🎮 Lógica principal y comandos seriales
│   ├── config.h                 # ⭐ Configuración completa (pines, PID, velocidades)
│   ├── sensores.h               # 📡 Gestión de 5 sensores IR con calibración
│   ├── motores.h                # 🚗 Control L298N con compensación de motores
│   ├── control_pid.h            # 🎯 Controlador PID adaptativo (3 modos)
│   └── nvs_config.h             # 💾 Persistencia en Flash (NVS)
│
├── pruebas/                     # 🧪 Herramientas de diagnóstico
│   ├── test_pines_adc.ino       # Test de lectura ADC de 5 sensores
│   ├── test_sensores.ino        # Calibración y diagnóstico avanzado
│   └── test_motores.ino         # Test de motores y compensación
│
├── docs/                        # 📚 Documentación técnica
│   ├── calibracion.md           # Guía detallada de calibración
│   └── tuning_pid.md            # Guía de ajuste fino del PID
│
├── DIAGRAMA_CONEXIONES.txt      # 📐 Diagrama ASCII de conexiones
├── ESP32_S3_HARDWARE.md         # 🔌 Especificaciones del ESP32-S3
└── diagrama_funciones.md        # 🗺️  Flujo de funciones del código
```

### Archivos Clave

| Archivo                              | Propósito                                                          |
| ------------------------------------ | ------------------------------------------------------------------- |
| **src/config.h**               | ⭐ Configuración central: pines GPIO, parámetros PID, velocidades |
| **src/sensores.h**             | Lectura ADC, calibración automática, cálculo de error ponderado  |
| **src/control_pid.h**          | PID con 3 modos adaptativos, anti-windup, filtro derivativo         |
| **src/main.cpp**               | Máquina de estados, comandos seriales, lógica de navegación      |
| **pruebas/test_pines_adc.ino** | Test rápido para verificar lecturas de los 5 sensores              |

## 📄 Licencia

MIT License - Ver [LICENSE](LICENSE) para más detalles.
