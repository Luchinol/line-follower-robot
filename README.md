# 🤖 Robot Seguidor de Línea - ESP32-S3

Sistema de seguimiento de línea autónomo con **ESP32-S3**, **5 sensores IR** y **control PID adaptativo**. Diseñado para máxima precisión y estabilidad en trayectorias complejas.

[![ESP32-S3](https://img.shields.io/badge/MCU-ESP32--S3-blue)](https://www.espressif.com/en/products/socs/esp32-s3)
[![PlatformIO](https://img.shields.io/badge/Platform-PlatformIO-orange)](https://platformio.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

## 🎯 Hardware Utilizado

| Componente                 | Modelo                            | Cantidad |
| -------------------------- | --------------------------------- | -------- |
| **Microcontrolador** | ESP32-S3 WROOM (FREENOVE)         | 1        |
| **Sensores IR**      | HW-511 (analógicos individuales) | 5        |
| **Puente H**         | L298N                             | 1        |
| **Motores DC**       | Con reductora 1:48                | 2        |
| **Batería**         | LiPo 3S 11.1V o 12V               | 1        |

### 📐 Especificaciones de Sensores

- **Tipo**: HW-511 (salida analógica)
- **Valores calibrados**:
  - BLANCO: ~100 ADC (12-bit)
  - NEGRO: ~2000 ADC
- **Resolución espacial**: 5 sensores con pesos **-5, -1, 0, +1, +5**
- **Rango de error**: **-500 a +500** (con resolución 10× mejorada)

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

  GPIO 6 → Sensor 1 (IZQ extremo)    | Peso: -5
  GPIO 5 → Sensor 2 (IZQ)            | Peso: -1
  GPIO 4 → Sensor 3 (CENTRO)         | Peso:  0
  GPIO 8 → Sensor 4 (DER)            | Peso: +1
  GPIO 7 → Sensor 5 (DER extremo)    | Peso: +5

BOTONES DE CONTROL:
  GPIO 0  (BOOT)  → Pausar/Reanudar
  GPIO 47         → Cambiar Modo
  GPIO 48 (LED)   → Parada de Emergencia

ALIMENTACIÓN:
  Batería 12V → L298N (+12V)
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
# USB CDC On Boot: "Disabled" (usa UART)
```

### 3. Calibrar Sensores

**Este es el paso más crítico para el correcto funcionamiento.**

```
1. Abrir el Monitor Serial (baudrate: 115200)
2. El robot inicia automáticamente en modo CALIBRANDO (8 segundos)
3. Durante ese tiempo, mover el robot manualmente asegurando que:
   - TODOS los 5 sensores pasen sobre la LÍNEA NEGRA
   - TODOS los 5 sensores pasen sobre la superficie BLANCA
   - Se recorra varias veces para capturar valores extremos
4. El sistema guarda automáticamente los valores min/max de cada sensor
5. Después de 8s, el robot pasa automáticamente a SIGUIENDO_LINEA
```

**Valores esperados después de calibración:**

- Sensores sobre BLANCO: ~100-300 ADC
- Sensores sobre NEGRO: ~1800-2200 ADC
- Umbral de detección: 74 (escala normalizada 0-100)

### 4. ¡A rodar!

El robot iniciará el seguimiento de línea automáticamente después de la calibración. Usa los comandos seriales o botones físicos para pausar, ajustar o reiniciar.

## 🎮 Comandos Seriales

### Comandos de Control

| Comando                    | Descripción                                  |
| -------------------------- | --------------------------------------------- |
| `0`                      | **Pausar** robot (atajo rápido)        |
| `1`                      | **Reanudar** operación (atajo rápido) |
| `pause` / `pausa`      | Pausar robot (detiene motores)                |
| `resume` / `continuar` | Reanudar operación                           |
| `stop` / `detener`     | Detener completamente                         |

### Configuración PID

| Comando                 | Descripción                                       |
| ----------------------- | -------------------------------------------------- |
| `p [Kp] [Ki] [Kd]`    | Ajustar PID RECTA (modo simplificado)              |
| `p [Kp] [Ki]`         | Modifica Kp y Ki (mantiene Kd actual)              |
| `p [Kp]`              | Modifica solo Kp                                   |
| `pc [Kp] [Ki] [Kd]`   | Ajustar parámetros del modo CURVA_CERRADA         |
| `pa` / `adaptativo` | Activar**modo PID ADAPTATIVO** (por defecto) |

### Otros Ajustes

| Comando              | Descripción                    |
| -------------------- | ------------------------------- |
| `v [velocidad]`    | Cambiar velocidad base (30-255) |
| `config` / `cfg` | Modo configuración interactiva |

### Sistema

| Comando                   | Descripción                        |
| ------------------------- | ----------------------------------- |
| `c` / `calibrar`      | Iniciar calibración de sensores    |
| `s` / `status`        | Mostrar estado del sistema completo |
| `r` / `reset`         | Reiniciar sistema                   |
| `d` / `diagnostico`   | Modo diagnóstico de hardware       |
| `h` / `?` / `ayuda` | Mostrar ayuda completa              |

### Persistencia (NVS - Flash)

| Comando                | Descripción                               |
| ---------------------- | ------------------------------------------ |
| `save` / `guardar` | Guardar config actual en Flash             |
| `load` / `cargar`  | Recargar config desde Flash                |
| `reset_config`       | Restaurar valores por defecto              |
| `nvs_info`           | Mostrar información de almacenamiento NVS |

### Comandos de Test

| Comando | Descripción                                              |
| ------- | --------------------------------------------------------- |
| `w`   | Test motores a VELOCIDAD_BASE (adelante)                  |
| `ts`  | Test sensores en tiempo real (presiona 'x' para salir)    |
| `tm`  | Test completo de motores (secuencia 4 pasos)              |
| `tp`  | Monitor PID en tiempo real (presiona 'x' para salir)      |
| `tc`  | Monitor detección de curvatura (presiona 'x' para salir) |

**Ejemplos de uso:**

```
p 0.5 0.0 0.3      # Ajusta PID RECTA (modo simplificado)
pc 1.5 0.0 0.8     # Ajusta PID CURVA_CERRADA
pa                 # Reactiva el modo adaptativo
v 130              # Ajusta velocidad a 130
save               # Guarda configuración en Flash (persiste después de apagar)
s                  # Muestra estado completo y configuración actual
```

## 🔧 Características Principales

### Control PID Adaptativo con 2 Modos (Sistema Simplificado)

El sistema ajusta automáticamente los parámetros PID según la curvatura detectada en tiempo real:

#### **Modo RECTA** (curvatura < 140)

- **Kp = 0.5** - Respuesta proporcional suave
- **Ki = 0.0** - Sin integral (evita wind-up)
- **Kd = 0.3** - Amortiguación ligera
- **Velocidad**: 100% de velocidad base (130 PWM por defecto)

#### **Modo CURVA CERRADA** (curvatura ≥ 140)

- **Kp = 1.5** - Respuesta proporcional agresiva
- **Ki = 0.0** - Sin integral (evita wind-up en curvas)
- **Kd = 0.8** - Amortiguación alta
- **Velocidad**: 50% de velocidad base (reducción dinámica)

### Algoritmo de Detección de Curvatura

**Fórmula híbrida con anticipación**:

```cpp
curvatura = |error_filtrado| × 0.7 + tasa_de_cambio × 0.3
```

**Componentes**:

- **Error absoluto (70%)**: Magnitud de desviación actual
- **Tasa de cambio (30%)**: Velocidad con que cambia el error (derivada)

**Ventaja**: Detecta curvas **antes** de que el error sea grande, permitiendo anticipación.

### Algoritmo de Detección de Línea

**5 sensores con pesos exponenciales:**

```
[-5]  [-1]  [0]  [+1]  [+5]
 S1    S2   S3   S4    S5
 IZQ   IZQ  CEN  DER   DER
```

**Cálculo de error ponderado con resolución mejorada (10×):**

```cpp
// Usa valores normalizados directamente (0-1000) sin pérdida de resolución
error = (Σ(valor[i] × peso[i]) × 10) / Σ(valor[i])
```

**Rango de error:** -500 (extremo izquierdo) a +500 (extremo derecho)

**Ventajas de pesos exponenciales (-5, -1, 0, +1, +5)**:

- ✅ Mayor sensibilidad en los extremos (detección temprana de curvas)
- ✅ Respuesta más suave en el centro
- ✅ Permite PID más agresivo sin oscilaciones
- ✅ Mejor performance en curvas cerradas
- ✅ **10× mejor resolución** vs versión anterior

### Filtrado y Suavizado

#### **Filtro de Error - Media Móvil Exponencial (EMA)**

```cpp
error_filtrado = 0.7 × error_nuevo + 0.3 × error_filtrado_anterior
```

- Reduce zigzagueo causado por ruido en sensores
- Respuesta rápida (70% nuevo) con suavizado (30% histórico)
- **Banda muerta**: Ignora errores < ±5 (evita correcciones por ruido)

#### **Amplificación Gradual de Corrección**

Para errores grandes (> 200), amplifica la corrección PID:

- Error 200-320: Amplificación gradual de 1.0× a 1.8×
- Error > 320: Amplificación máxima 1.8×
- Transición suave sin saltos bruscos

#### **Modo PIVOTE para Curvas Extremas**

Cuando `|error| > 350` (solo sensores extremos detectan línea):

- Rueda interior: 10% (pivote asistido)
- Rueda exterior: 90% velocidad
- Permite giros de hasta 180° (horquillas)
- Radio de giro: ~20cm

### Sistema de Recuperación de Línea (3 Fases)

Cuando se pierde la línea, el robot ejecuta una estrategia inteligente de 3 fases:

#### **Fase 1: Tolerancia Inicial (0-1500ms)**

- Mantiene la **última corrección PID conocida**
- Velocidad reducida (VELOCIDAD_MIN = 35)
- Continúa la curva que probablemente causó la pérdida

```cpp
vel_izq = VELOCIDAD_MIN + ultima_correccion
vel_der = VELOCIDAD_MIN - ultima_correccion
```

#### **Fase 2: Retroceso Inteligente (1500-2500ms)**

- Retrocede girando hacia donde estaba la línea
- Dirección basada en último error significativo (|error| > 100)
- Factor de giro: 0.6 (rueda interior al 60%)

#### **Fase 3: Búsqueda Activa (2500-3500ms)**

- Gira sobre su eje alternando dirección cada 1 segundo
- Velocidad de búsqueda: 120 PWM
- Timeout total: 3.5 segundos → DETENIDO

### Compensación de Motores

Los motores DC nunca son idénticos. El sistema incluye factores de compensación calibrados:

```cpp
FACTOR_MOTOR_DERECHO   = 1.00   // Baseline (motor de referencia)
FACTOR_MOTOR_IZQUIERDO = 1.07   // +7% compensación (motor más débil)
```

**Calibración actualizada**: 2025-11-12

- Desviación en 3m: < 5cm
- Desviación angular: 1.2° (objetivo: <2°)
- Comportamiento estable en rango 100-200 PWM

### Mapeo PWM Inteligente

**Problema**: Motores tienen zona muerta 0-51% PWM (no giran)

**Solución**: Mapeo automático

- Usuario: 1-255 → PWM real: 130-255 (51%-100%)
- Elimina zona muerta completamente
- Control lineal y predecible
- **PWM_MIN_EFECTIVO = 130** (51% de 255)

### Persistencia de Configuración (NVS)

La configuración se guarda en **memoria Flash** y sobrevive a:

- ✅ Apagados del robot
- ✅ Reinicios por software
- ✅ Pérdida de energía

**Valores persistentes**:

- Velocidad base
- Parámetros PID (Kp, Ki, Kd) para ambos modos
- Calibración de sensores

**Ciclos de escritura**: ~100,000 por sector

## 🎛️ Botones Físicos

| GPIO | Botón         | Función             |
| ---- | -------------- | -------------------- |
| 0    | BOOT (onboard) | Pausar/Reanudar      |
| 47   | Externo        | Cambiar Modo         |
| 48   | LED WS2812     | Parada de Emergencia |

Todos los botones tienen debounce por software (50ms).

## 🐛 Troubleshooting

| Problema                        | Solución Sugerida                                                                               |
| ------------------------------- | ------------------------------------------------------------------------------------------------ |
| **Oscila mucho en recta** | `Kp` muy alto. Prueba: `p 0.4 0.0 0.3` y `save`                                            |
| **Se sale en curvas**     | Velocidad alta o `Kp` bajo. Prueba: `v 100` o `pc 2.0 0.0 1.0`                             |
| **Movimiento errático**  | Mala calibración. Ejecuta `c` y asegúrate de que TODOS los sensores vean blanco y negro      |
| **Un motor más lento**   | Ajusta `FACTOR_MOTOR_DERECHO` o `FACTOR_MOTOR_IZQUIERDO` en `src/config.h`                 |
| **No responde comandos**  | Verifica baudrate:`115200` y terminador de línea: `NL & CR`                                 |
| **Se pierde en curvas**   | Aumenta agresividad de curvas:`pc 2.5 0.0 1.5`                                                 |
| **No detecta línea**     | Verifica umbral de detección (debe ser ~74). Usa comando `ts` para ver valores en tiempo real |
| **Pivote oscilante**      | Robot entra y sale de pivote rápidamente. Reduce velocidad:`v 100`                            |

## 📂 Estructura del Proyecto

```
CarritoSeguidor/
├── README.md                    # 📖 Esta guía completa
├── platformio.ini               # ⚙️  Configuración de PlatformIO
├── LICENSE                      # 📄 Licencia MIT
├── .gitignore                   # 🚫 Archivos excluidos de Git
│
├── src/                         # 💻 Código fuente principal
│   ├── main.cpp                 # 🎮 Lógica principal (~1800 líneas)
│   ├── config.h                 # ⭐ Configuración completa (436 líneas)
│   ├── sensores.h               # 📡 Gestión de 5 sensores IR (~250 líneas)
│   ├── motores.h                # 🚗 Control L298N (~400 líneas)
│   ├── control_pid.h            # 🎯 Controlador PID adaptativo (~350 líneas)
│   └── nvs_config.h             # 💾 Persistencia en Flash (~200 líneas)
│
├── PRESENTACION_PROYECTO.md     # 📊 Documentación técnica detallada
├── PRESENTACION_PROYECTO.pptx   # 📊 Presentación PowerPoint (31 diaps)
├── generar_pptx.py             # 🐍 Script generador de presentaciones
│
└── .claude/                     # 🤖 Configuración de Claude Code
```

### Archivos Clave

| Archivo                     | Propósito                                                          |
| --------------------------- | ------------------------------------------------------------------- |
| **src/config.h**      | ⭐ Configuración central: pines GPIO, parámetros PID, velocidades |
| **src/main.cpp**      | Máquina de estados, comandos seriales, lógica de seguimiento      |
| **src/sensores.h**    | Lectura ADC, calibración automática, cálculo de error ponderado  |
| **src/control_pid.h** | PID con 2 modos adaptativos, anti-windup, filtro derivativo         |
| **src/motores.h**     | Control PWM, compensación de motores, mapeo de zona muerta         |
| **src/nvs_config.h**  | Almacenamiento persistente en Flash (NVS)                           |

## 🔬 Características Técnicas Avanzadas

### Máquina de Estados (8 estados)

```
CALIBRANDO (8s auto)
    ↓
SIGUIENDO_LINEA ⟷ PERDIDA_LINEA (1.5s) → BUSCANDO_LINEA (1s) → DETENIDO
    ↓                                                                ↑
PAUSADO ⟷ CONFIGURACION                                             │
    ↓                                                                │
DIAGNOSTICO ─────────────────────────────────────────────────────────┘
```

### Control PID Completo

**Ecuación implementada**:

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de(t)/dt
```

**Protecciones**:

- ✅ Anti-windup en integral (límite: ±1000)
- ✅ Filtro derivativo (α=0.2) para reducir ruido
- ✅ Límite de salida: ±100 o 80% de velocidad actual
- ✅ Reset automático de integral al cambiar de modo
- ✅ Reset periódico de errorMaximo cada 10000 ciclos

### Estadísticas en Tiempo Real

- Error promedio (media móvil)
- Error máximo observado (con reset periódico)
- Ciclos de procesamiento PID
- Tiempo de operación
- Modo PID actual (RECTA/CURVA_CERRADA)
- Velocidad de motores

## 📊 Parámetros de Configuración (Estado Actual)

### Velocidades (config.h)

```cpp
VELOCIDAD_BASE   = 130   // Velocidad en recta (conservadora)
VELOCIDAD_MIN    = 35    // Velocidad mínima útil
VELOCIDAD_MAX    = 255   // Velocidad máxima
```

### PID por Defecto (Sistema Simplificado)

```cpp
// Modo RECTA
PID_RECTA_DEFAULT_KP = 0.5
PID_RECTA_DEFAULT_KI = 0.0
PID_RECTA_DEFAULT_KD = 0.3

// Modo CURVA CERRADA
PID_CERRADA_DEFAULT_KP = 1.5
PID_CERRADA_DEFAULT_KI = 0.0
PID_CERRADA_DEFAULT_KD = 0.8
```

### Umbrales de Curvatura

```cpp
UMBRAL_CURVA_CERRADA  = 140   // Transición recta → curva cerrada
```

### Timeouts de Recuperación

```cpp
TIMEOUT_PERDIDA_LINEA = 1500 ms  // Fase 1: Mantiene dirección
TIMEOUT_RETROCESO     = 2500 ms  // Fase 2: Retroceso inteligente
TIMEOUT_BUSQUEDA      = 3500 ms  // Fase 3: Búsqueda activa
```

### Pivote para Curvas Extremas

```cpp
UMBRAL_GIRO_CRITICO       = 350   // Error para activar pivote
VELOCIDAD_PIVOTE_INTERIOR = 10    // 10% PWM (rueda lenta)
VELOCIDAD_PIVOTE_EXTERIOR = 90    // 90% PWM (rueda rápida)
```

### Ciclo de Control

```cpp
DELAY_CICLO_CONTROL = 5 ms      // ~200Hz de frecuencia
```

### Compensación de Motores

```cpp
FACTOR_MOTOR_DERECHO   = 1.00   // Motor de referencia
FACTOR_MOTOR_IZQUIERDO = 1.07   // +7% compensación
```

### PWM Físico

```cpp
PWM_MIN_EFECTIVO = 130   // 51% de 255 (elimina zona muerta)
PWM_MAX_EFECTIVO = 255   // 100% máximo
PWM_FREQUENCY    = 5000  // 5 kHz
```

### Factor de Velocidad en Curvas

```cpp
FACTOR_VEL_CURVA_CERRADA = 0.50  // Reduce a 50% en curvas cerradas
```

## 💡 Tips de Uso

### Para Pistas Rápidas

```
v 180              # Aumenta velocidad
p 0.4 0.0 0.3      # PID más suave en rectas
save
```

### Para Pistas con Curvas Cerradas

```
v 100              # Reduce velocidad base
pc 2.0 0.0 1.2     # PID más agresivo en curvas
save
```

### Para Debugging

```
ts                 # Monitor de sensores en tiempo real
tc                 # Monitor de detección de curvatura
tp                 # Monitor de cálculos PID
```

### Para Resetear Todo

```
reset_config       # Borra configuración guardada
c                  # Recalibra sensores
```

## 🏆 Características Destacadas

✅ **PID Adaptativo Simplificado**: Sistema de 2 modos automáticos según curvatura
✅ **Detección Anticipatoria**: Usa tasa de cambio para predecir curvas
✅ **Recuperación Inteligente**: Estrategia de 3 fases con memoria de dirección
✅ **Configuración Persistente**: NVS guarda parámetros en Flash
✅ **Compensación de Hardware**: Equaliza motores desiguales (calibrado a 7%)
✅ **Comandos Completos**: 30+ comandos para control total
✅ **Telemetría Avanzada**: Estadísticas en tiempo real con reset periódico
✅ **Filtros Múltiples**: EMA en error, suavizado en derivada
✅ **Modo Pivote**: Giros de 180° en curvas extremas (10%/90%)
✅ **Interfaz Interactiva**: Ajustes en runtime sin recompilar
✅ **Resolución Mejorada**: 10× mejor precisión en sensores (0-1000 directo)

## 📊 Comparativa ESP32-S3 vs Arduino Uno

| Característica   | ESP32-S3          | Arduino Uno      | Factor                  |
| ----------------- | ----------------- | ---------------- | ----------------------- |
| **CPU**     | 240 MHz (32-bit)  | 16 MHz (8-bit)   | 15×                    |
| **RAM**     | 512 KB            | 2 KB             | 256×                   |
| **Flash**   | 8 MB              | 32 KB            | 250×                   |
| **ADC**     | 20 canales 12-bit | 6 canales 10-bit | 3.3× + 4× resolución |
| **PWM**     | 16 canales        | 6 canales        | 2.6×                   |
| **UART**    | 3 hardware        | 1 hardware       | 3×                     |
| **FPU**     | Sí (hardware)    | No (software)    | 625× más rápido      |
| **WiFi/BT** | Integrados        | No               | ✅                      |
| **Precio**  | $8-12 | $20-25    | 50% más barato  |                         |

**Veredicto**: Este proyecto **requiere ESP32-S3** debido a:

- ✅ Código 359 KB (no cabe en Arduino: 32 KB)
- ✅ RAM suficiente (20 KB usado / 512 KB disponible)
- ✅ FPU para PID flotante eficiente
- ✅ 16 canales PWM sin conflictos

Ver [PRESENTACION_PROYECTO.md](PRESENTACION_PROYECTO.md) para análisis detallado completo.

## 🚀 Próximas Mejoras Posibles

Ver archivo [PRESENTACION_PROYECTO.md](PRESENTACION_PROYECTO.md) - Diapositiva 21 para el plan completo de mejoras sugeridas:

1. 📡 **Control remoto WiFi** (AP o STA)
2. 📱 **App móvil Bluetooth LE**
3. 📊 **Telemetría IoT** (MQTT, HTTP)
4. 🔄 **Actualización OTA** firmware
5. 📷 **Cámara OV2640** (visión artificial)
6. 🖥️ **Display OLED/TFT** (estado visual)
7. 📍 **GPS** (tracking de posición)
8. 🎮 **Acelerómetro/Giroscopio** (IMU)
9. 💾 **Logger SD Card** (datos de carrera)
10. 🤖 **Multi-robot** (comunicación ESP-NOW)

## 📄 Licencia

MIT License - Ver [LICENSE](LICENSE) para más detalles.

## 👨‍💻 Autor

**LUCHIN-OPRESORCL**

- Versión: 2.0.1
- Fecha última actualización: 2025-11-11
- GitHub: Luchinol

---

## 📚 Documentación Adicional

- **Presentación técnica completa**: [PRESENTACION_PROYECTO.md](PRESENTACION_PROYECTO.md)
- **PowerPoint (31 diapositivas)**: [PRESENTACION_PROYECTO.pptx](PRESENTACION_PROYECTO.pptx)
- **Comparativa ESP32 vs Arduino**: Ver Diapositivas 11-20 de la presentación
- **Funciones lógicas detalladas**: Ver Diapositivas 3-10 de la presentación

---

**⚡ Proyecto actualizado con las últimas mejoras y configuraciones optimizadas**
