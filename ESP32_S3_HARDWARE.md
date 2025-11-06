# Configuración Hardware ESP32-S3 WROOM FREENOVE

Este documento detalla el hardware específico para este proyecto, enfocado en la placa **ESP32-S3 WROOM de FREENOVE** y la configuración actual de sensores.

## 📦 Hardware Utilizado

### Microcontrolador: ESP32-S3 WROOM (FREENOVE)

#### Especificaciones Clave
- **Modelo**: ESP32-S3-WROOM-1 (FREENOVE)
- **Procesador**: Xtensa dual-core 32-bit LX7 a 240 MHz
- **PSRAM**: 8MB integrado ⭐
- **USB**: Nativo (no requiere chip UART externo) ⭐

#### Características Específicas de la Placa FREENOVE

- **PSRAM de 8MB integrado**: Ocupa los GPIOs 35-37.
- **USB Nativo**: Usa los GPIOs 19 y 20. **No tocar estos pines**.
- **LED RGB (WS2812)**: Integrado en el GPIO 48.
- **Compatibilidad con Arduino IDE**: Usar la placa "ESP32S3 Dev Module".

### Sensores IR: 3x HW-511 (Individuales)

La configuración actual del proyecto utiliza 3 sensores infrarrojos individuales.

```
┌────────────────────────────────────────┐
│         Sensor HW-511                  │
├────────────────────────────────────────┤
│ Chip: LM393 (comparador)               │
│ Alimentación: 3.3V - 5V                │
│ Salida: Analógica (0-3.3V) y Digital   │
│ Distancia óptima: 2-10 mm              │
│ Ajuste: Potenciómetro de sensibilidad │
└────────────────────────────────────────┘

Pinout del HW-511:
  VCC → 5V o 3.3V
  GND → GND
  D0  → Salida Digital (no usada en este proyecto)
  A0  → Salida Analógica → Conectar a los pines ADC del ESP32-S3
```

> **Nota sobre la configuración anterior**: Los comentarios en el código y otros documentos pueden hacer referencia a una configuración más compleja con 10 sensores (un array lejano y uno cercano). Esa configuración está **deshabilitada** en favor de esta configuración más simple y robusta de 3 sensores.

## 🔌 Mapeo de Pines (Configuración Actual)

### Conexiones de Motores (L298N)

```
┌─────────────────────────────────────────────────────────────┐
│  MOTOR DERECHO                                              │
├────────────┬────────────┬───────────────────────────────────┤
│ ESP32-S3   │  L298N     │  Función                          │
├────────────┼────────────┼───────────────────────────────────┤
│ GPIO 12    │  ENA       │  PWM para control de velocidad    │
│ GPIO 11    │  IN1       │  Control de dirección             │
│ GPIO 18    │  IN2       │  Control de dirección             │
└────────────┴────────────┴───────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  MOTOR IZQUIERDO                                            │
├────────────┬────────────┬───────────────────────────────────┤
│ ESP32-S3   │  L298N     │  Función                          │
├────────────┼────────────┼───────────────────────────────────┤
│ GPIO 13    │  ENB       │  PWM para control de velocidad    │
│ GPIO 14    │  IN3       │  Control de dirección             │
│ GPIO 21    │  IN4       │  Control de dirección             │
└────────────┴────────────┴───────────────────────────────────┘
```

### Conexiones de Sensores IR (3x HW-511)

```
┌────────────────────────────────────────────────────────────────┐
│  ARRAY DE 3 SENSORES                                           │
├──────────┬────────────┬────────────────┬──────────────────────┤
│ ESP32-S3 │  Sensor    │  Posición      │  ADC                 │
├──────────┼────────────┼────────────────┼──────────────────────┤
│ GPIO 3   │  HW-511 #1 │  Izquierdo     │  ADC1_CH2            │
│ GPIO 4   │  HW-511 #2 │  Centro        │  ADC1_CH3            │
│ GPIO 5   │  HW-511 #3 │  Derecho       │  ADC1_CH4            │
└──────────┴────────────┴────────────────┴──────────────────────┘

Conexión de cada HW-511:
  VCC → 5V (puede ser desde el L298N o el pin 5V del ESP32)
  GND → GND común
  A0  → GPIO correspondiente (3, 4 o 5)
```

### Alimentación

```
Batería (7.4V LiPo o 6xAA)
   ↓
Switch ON/OFF
   ↓
L298N (Entrada +12V)
   ↓
   ├─► Alimentación de potencia para los motores
   ↓
Regulador de 5V del L298N (Salida +5V)
   ↓
   ├─► ESP32-S3 (pin VIN)
   └─► Sensores (pin VCC)

GND COMÚN: Es CRÍTICO conectar los terminales GND de la batería, L298N, ESP32 y todos los sensores juntos.
```

## ⚙️ Ajuste de Sensores HW-511

Cada módulo HW-511 tiene un potenciómetro para ajustar su sensibilidad. Este ajuste es fundamental.

**Procedimiento de ajuste:**
1. Coloca un sensor sobre la superficie **BLANCA** de tu pista.
2. Gira el potenciómetro lentamente hasta que el LED del módulo se apague.
3. Coloca el mismo sensor sobre la línea **NEGRA**.
4. El LED debería encenderse.
5. Si no es así, reajusta el potenciómetro hasta encontrar el punto donde el LED se encienda sobre el negro y se apague sobre el blanco.
6. Repite el proceso para los 3 sensores.

--- 
**Última actualización:** 2025-11-05 (simplificado para 3 sensores)