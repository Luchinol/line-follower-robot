# 🤖 Robot Seguidor de Línea - ESP32-S3

Sistema de seguimiento de línea autónomo con **ESP32-S3**, **5 sensores IR** y **control PID adaptativo**. Diseñado para máxima precisión y estabilidad en trayectorias complejas.

## 🎯 Hardware Utilizado

| Componente                 | Modelo                            | Cantidad |
| -------------------------- | --------------------------------- | -------- |
| **Microcontrolador** | ESP32-S3 WROOM (FREENOVE)         | 1        |
| **Sensores IR**      | HW-511 (analógicos individuales) | 5        |
| **Puente H**         | L298N                             | 1        |
| **Motores DC**       | Con reductora 1:48                | 2        |
| **Batería**         | LiPo 2S 7.4V o 12V                | 1        |

### 📐 Especificaciones de Sensores

- **Tipo**: HW-511 (salida analógica)
- **Valores calibrados**:
  - BLANCO: ~100 ADC (12-bit)
  - NEGRO: ~2000 ADC
- **Resolución espacial**: 5 sensores con pesos **-5, -1, 0, +1, +5**
- **Rango de error**: **-400 a +400** (pesos exponenciales)

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

| Comando              | Descripción                                                 |
| -------------------- | ----------------------------------------------------------- |
| `0`                | **Pausar** robot (atajo rápido)                             |
| `1`                | **Reanudar** operación (atajo rápido)                       |
| `pause` / `pausa` | Pausar robot (detiene motores)                              |
| `resume` / `continuar` | Reanudar operación                                    |
| `stop` / `detener` | Detener completamente                                      |

### Configuración PID

| Comando                      | Descripción                                          |
| ---------------------------- | ---------------------------------------------------- |
| `p [Kp] [Ki] [Kd]`         | Ajustar PID en **modo MANUAL** (desactiva adaptativo) |
| `p [Kp] [Ki]`              | Modifica Kp y Ki (mantiene Kd actual)                |
| `p [Kp]`                   | Modifica solo Kp                                     |
| `p recta [Kp] [Ki] [Kd]`  | Ajustar parámetros del modo RECTA                    |
| `p suave [Kp] [Ki] [Kd]`  | Ajustar parámetros del modo CURVA_SUAVE              |
| `p cerrada [Kp] [Ki] [Kd]`| Ajustar parámetros del modo CURVA_CERRADA            |
| `pa` / `adaptativo`       | Activar **modo PID ADAPTATIVO**                      |

### Otros Ajustes

| Comando         | Descripción                                |
| --------------- | ------------------------------------------ |
| `v [velocidad]`| Cambiar velocidad base (0-255)             |
| `config` / `cfg`| Modo configuración interactiva            |

### Sistema

| Comando              | Descripción                                    |
| -------------------- | ---------------------------------------------- |
| `c` / `calibrar`   | Iniciar calibración de sensores                |
| `s` / `status`     | Mostrar estado del sistema completo            |
| `r` / `reset`      | Reiniciar sistema                              |
| `d` / `diagnostico`| Modo diagnóstico de hardware                   |
| `h` / `?` / `ayuda`| Mostrar ayuda completa                         |

### Persistencia (NVS - Flash)

| Comando           | Descripción                                       |
| ----------------- | ------------------------------------------------- |
| `save` / `guardar`| Guardar config actual en Flash                    |
| `load` / `cargar` | Recargar config desde Flash                       |
| `reset_config`    | Restaurar valores por defecto                     |
| `nvs_info`        | Mostrar información de almacenamiento NVS         |

### Comandos de Test

| Comando | Descripción                                           |
| ------- | ----------------------------------------------------- |
| `w`     | Test motores a VELOCIDAD_BASE (adelante)              |
| `ts`    | Test sensores en tiempo real (presiona 'x' para salir)|
| `tm`    | Test completo de motores (secuencia 4 pasos)          |
| `tp`    | Monitor PID en tiempo real (presiona 'x' para salir)  |
| `tc`    | Monitor detección de curvatura (presiona 'x' para salir)|

**Ejemplos de uso:**

```
p 1.5 0.01 0.8     # Ajusta PID manualmente (desactiva modo adaptativo)
p recta 1.0 0.005 0.5  # Ajusta solo el modo RECTA (mantiene adaptativo)
pa                 # Reactiva el modo adaptativo
v 150              # Reduce velocidad a 150
save               # Guarda configuración en Flash (persiste después de apagar)
s                  # Muestra estado completo y configuración actual
```

## 🔧 Características Principales

### Control PID Adaptativo con 3 Modos

El sistema ajusta automáticamente los parámetros PID según la curvatura detectada en tiempo real:

#### **Modo RECTA** (curvatura < 80)

- **Kp = 1.0** - Respuesta proporcional suave
- **Ki = 0.005** - Corrección integral mínima
- **Kd = 0.5** - Amortiguación moderada
- **Velocidad**: 100% de velocidad base (120 PWM por defecto)

#### **Modo CURVA SUAVE** (80 ≤ curvatura < 140)

- **Kp = 1.8** - Mayor respuesta proporcional
- **Ki = 0.02** - Integral moderada
- **Kd = 1.0** - Mayor amortiguación
- **Velocidad**: 85% de velocidad base

#### **Modo CURVA CERRADA** (curvatura ≥ 140)

- **Kp = 2.5** - Respuesta muy agresiva
- **Ki = 0.0** - Sin integral (evita wind-up en curvas)
- **Kd = 1.2** - Amortiguación máxima
- **Velocidad**: 60% de velocidad base

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

**Cálculo de error ponderado:**

```cpp
error = Σ(valor_normalizado[i] × peso[i]) × 100 / Σ(valor_normalizado[i])
```

**Rango de error:** -400 (extremo izquierdo) a +400 (extremo derecho)

**Ventajas de pesos exponenciales (-5, -1, 0, +1, +5)**:

- ✅ Mayor sensibilidad en los extremos (detección temprana de curvas)
- ✅ Respuesta más suave en el centro
- ✅ Permite PID más agresivo sin oscilaciones
- ✅ Mejor performance en curvas cerradas

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

- Error 200-320: Amplificación gradual de 1.0x a 1.8x
- Error > 320: Amplificación máxima 1.8x
- Transición suave sin saltos bruscos

#### **Modo PIVOTE para Curvas Extremas**

Cuando `|error| > 350` (solo sensores extremos detectan línea):

- Rueda interior: 0% (DETENIDA) - pivote puro
- Rueda exterior: 80% velocidad
- Permite giros de hasta 180° (horquillas)

### Sistema de Recuperación de Línea (3 Fases)

Cuando se pierde la línea, el robot ejecuta una estrategia inteligente de 3 fases:

#### **Fase 1: Tolerancia Inicial (0-1500ms)**

- Mantiene la **última corrección PID conocida**
- Velocidad reducida (VELOCIDAD_MIN = 30)
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

Los motores DC nunca son idénticos. El sistema incluye factores de compensación:

```cpp
FACTOR_MOTOR_DERECHO   = 1.00   // Baseline (motor de referencia)
FACTOR_MOTOR_IZQUIERDO = 1.13   // +13% compensación (motor más débil)
```

**Calibración realizada**: 2025-11-07
- Desviación en 3m: < 5cm
- Desviación angular: 1.2° (objetivo: <2°)

### Mapeo PWM Inteligente

**Problema**: Motores tienen zona muerta 0-40% PWM (no giran)

**Solución**: Mapeo automático
- Usuario: 0-255 → PWM real: 0 o 102-255 (40%-100%)
- Elimina zona muerta completamente
- Control lineal y predecible

### Persistencia de Configuración (NVS)

La configuración se guarda en **memoria Flash** y sobrevive a:
- ✅ Apagados del robot
- ✅ Reinicios por software
- ✅ Pérdida de energía

**Valores persistentes**:
- Velocidad base
- Parámetros PID (Kp, Ki, Kd)

**Ciclos de escritura**: ~100,000 por sector

## 🎛️ Botones Físicos

| GPIO | Botón              | Función                |
| ---- | ------------------ | ---------------------- |
| 0    | BOOT (onboard)     | Pausar/Reanudar        |
| 47   | Externo            | Cambiar Modo           |
| 48   | LED WS2812         | Parada de Emergencia   |

Todos los botones tienen debounce por software (50ms).

## 🐛 Troubleshooting

| Problema                           | Solución Sugerida                                                                                                |
| ---------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| **Oscila mucho en recta** | `Kp` muy alto. Prueba: `p recta 0.8 0.005 0.5` y `save`                              |
| **Se sale en curvas**    | Velocidad alta o `Kp` bajo. Prueba: `v 100` o `p suave 2.0 0.02 1.0`                    |
| **Movimiento errático**  | Mala calibración. Ejecuta `c` y asegúrate de que TODOS los sensores vean blanco y negro |
| **Un motor más lento** | Ajusta `FACTOR_MOTOR_DERECHO` o `FACTOR_MOTOR_IZQUIERDO` en `src/config.h`              |
| **No responde comandos**   | Verifica baudrate: `115200` y terminador de línea: `NL & CR`                |
| **Se pierde en curvas** | Aumenta agresividad de curvas: `p cerrada 3.0 0.0 1.5`                                   |
| **No detecta línea** | Verifica umbral de detección (debe ser ~74). Usa comando `ts` para ver valores en tiempo real |

## 📂 Estructura del Proyecto

```
CarritoSeguidor/
├── README.md                    # 📖 Esta guía completa
├── platformio.ini               # ⚙️  Configuración de PlatformIO
├── LICENSE                      # 📄 Licencia MIT
├── .gitignore                   # 🚫 Archivos excluidos de Git
│
├── src/                         # 💻 Código fuente principal
│   ├── main.cpp                 # 🎮 Lógica principal (1660 líneas)
│   ├── config.h                 # ⭐ Configuración completa (444 líneas)
│   ├── sensores.h               # 📡 Gestión de 5 sensores IR (246 líneas)
│   ├── motores.h                # 🚗 Control L298N (659 líneas)
│   ├── control_pid.h            # 🎯 Controlador PID adaptativo (464 líneas)
│   └── nvs_config.h             # 💾 Persistencia en Flash (258 líneas)
│
└── .claude/                     # 🤖 Configuración de Claude Code
```

### Archivos Clave

| Archivo                  | Propósito                                                          |
| ------------------------ | ------------------------------------------------------------------- |
| **src/config.h**   | ⭐ Configuración central: pines GPIO, parámetros PID, velocidades |
| **src/main.cpp**   | Máquina de estados, comandos seriales, lógica de seguimiento      |
| **src/sensores.h** | Lectura ADC, calibración automática, cálculo de error ponderado  |
| **src/control_pid.h** | PID con 3 modos adaptativos, anti-windup, filtro derivativo    |
| **src/motores.h**  | Control PWM, compensación de motores, mapeo de zona muerta        |
| **src/nvs_config.h** | Almacenamiento persistente en Flash (NVS)                       |

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

### Estadísticas en Tiempo Real

- Error promedio (media móvil)
- Error máximo observado
- Ciclos de procesamiento
- Tiempo de operación
- Modo PID actual
- Velocidad de motores

## 📊 Parámetros de Configuración

### Velocidades (config.h)

```cpp
VELOCIDAD_BASE   = 120   // Velocidad en recta (conservadora)
VELOCIDAD_MIN    = 30    // Velocidad mínima útil
VELOCIDAD_MAX    = 255   // Velocidad máxima
```

### Umbrales de Curvatura

```cpp
UMBRAL_CURVA_SUAVE    = 80    // Transición recta → curva suave
UMBRAL_CURVA_CERRADA  = 140   // Transición curva suave → cerrada
```

### Timeouts de Recuperación

```cpp
TIMEOUT_PERDIDA_LINEA = 1500 ms  // Fase 1: Mantiene dirección
TIMEOUT_RETROCESO     = 2500 ms  // Fase 2: Retroceso inteligente
TIMEOUT_BUSQUEDA      = 3500 ms  // Fase 3: Búsqueda activa
```

### Ciclo de Control

```cpp
DELAY_CICLO_CONTROL = 5 ms      // ~200Hz de frecuencia
```

## 💡 Tips de Uso

### Para Pistas Rápidas

```
v 180              # Aumenta velocidad
p recta 0.8 0.005 0.4    # PID más suave en rectas
save
```

### Para Pistas con Curvas Cerradas

```
v 100              # Reduce velocidad base
p cerrada 3.0 0.0 1.8    # PID muy agresivo en curvas
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

✅ **PID Adaptativo**: Primer sistema con 3 modos automáticos según curvatura
✅ **Detección Anticipatoria**: Usa tasa de cambio para predecir curvas
✅ **Recuperación Inteligente**: Estrategia de 3 fases con memoria de dirección
✅ **Configuración Persistente**: NVS guarda parámetros en Flash
✅ **Compensación de Hardware**: Equaliza motores desiguales
✅ **Comandos Completos**: 30+ comandos para control total
✅ **Telemetría Avanzada**: Estadísticas en tiempo real
✅ **Filtros Múltiples**: EMA en error, suavizado en derivada
✅ **Modo Pivote**: Giros de 180° en curvas extremas
✅ **Interfaz Interactiva**: Ajustes en runtime sin recompilar

## 📄 Licencia

MIT License - Ver [LICENSE](LICENSE) para más detalles.

## 👨‍💻 Autor

**LUCHIN-OPRESORCL**
Versión: 2.0.0
Fecha: 2025-11-07
