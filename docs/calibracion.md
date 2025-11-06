# Guía Completa de Calibración de Sensores IR

La calibración es el paso **MÁS IMPORTANTE** para que tu robot siga la línea correctamente. Una mala calibración hará que el robot se comporte erráticamente sin importar qué tan bien ajustes el PID.

> 📌 **Configuración actual:** 5 sensores HW-511 analógicos (GPIO 6, 5, 4, 8, 7)

## 📋 Tabla de Contenidos

1. [¿Por qué calibrar?](#1-por-qué-calibrar)
2. [Preparación](#2-preparación)
3. [Proceso de Calibración](#3-proceso-de-calibración)
4. [Verificación de Calibración](#4-verificación-de-calibración)
5. [Problemas Comunes](#5-problemas-comunes)
6. [Herramientas de Diagnóstico](#6-herramientas-de-diagnóstico)

---

## 1. ¿Por qué calibrar?

### 1.1 El Problema

Cada sensor IR HW-511 es ligeramente diferente y las condiciones ambientales varían. Sin calibración, el robot no puede interpretar las lecturas de forma fiable.

```
// Ejemplo sin calibración:
Sensor 1 (GPIO6) sobre BLANCO lee: 120 ADC
Sensor 3 (GPIO4) sobre BLANCO lee: 180 ADC

Sensor 1 sobre NEGRO lee: 1950 ADC
Sensor 3 sobre NEGRO lee: 2150 ADC

Problema: ¡El robot no tiene una referencia clara de qué es blanco y qué es negro!
```

### 1.2 La Solución

La calibración "enseña" al robot el rango de lecturas para cada sensor, normalizando los valores a una escala de 0 a 1000 para que puedan ser comparados de forma consistente.

```cpp
// Lógica de calibración:
Para cada uno de los 5 sensores:
1. Encontrar el valor MÍNIMO (sobre blanco) → ~100-300 ADC
2. Encontrar el valor MÁXIMO (sobre negro) → ~1800-2200 ADC

// En tiempo real, se normaliza:
valor_normalizado = map(valor_actual, min_calibrado, max_calibrado, 0, 1000)
```

### 1.3 Valores Esperados (HW-511)

| Superficie | Valor ADC (12-bit) | Descripción |
|------------|-------------------|-------------|
| **BLANCO** | ~100-300 | Máxima reflexión IR |
| **NEGRO** | ~1800-2200 | Mínima reflexión IR |
| **Rango ideal** | >1500 | Diferencia entre negro y blanco |

---

## 2. Preparación

### 2.1 Requisitos de la Pista

- **Fondo:** Blanco y MATE. Evita superficies brillantes (pueden saturar los sensores)
- **Línea:** Negra y MATE. Cinta aislante negra funciona bien
  - **Ancho recomendado:** 1.5 - 2.5 cm
  - **Material:** Vinilo negro mate o cinta aislante
- **Iluminación:** Luz artificial uniforme
  - ⚠️ **EVITA luz solar directa** (interfiere con IR)

### 2.2 Montaje Físico de Sensores

**Configuración del array (5 sensores):**
```
[S1]  [S2]  [S3]  [S4]  [S5]
IZQ+2 IZQ+1 CEN  DER+1 DER+2
GPIO6 GPIO5 GPIO4 GPIO8 GPIO7
```

**Verificación pre-calibración:**
- ✅ **Altura:** 3-8 mm del suelo (óptimo: 5mm para HW-511)
- ✅ **Alineación:** TODOS los sensores a la misma altura y paralelos
- ✅ **Separación:** Distribución uniforme en el ancho del robot
- ✅ **Conexiones:**
  - VCC → 5V (desde L298N)
  - GND → GND común
  - OUT → GPIOs correspondientes
- ✅ **Pines ADC:** NO usar `pinMode()` (el código ya está corregido)

### 2.3 Test Rápido Pre-Calibración

Ejecuta el test de pines para verificar lecturas:

```bash
# Desde PlatformIO, sube el test
pio run -t upload --environment test_pines_adc
```

Deberías ver valores fluctuantes para cada sensor. Si ves `0` o `4095` constante, hay un problema de conexión.

---

## 3. Proceso de Calibración

### 3.1 Calibración Automática (8 segundos)

#### Paso 1: Iniciar Calibración

1. Conecta el ESP32-S3 a tu PC
2. Abre el **Monitor Serial** (baudrate: 115200)
3. Carga el programa principal (`src/main.cpp`)
4. Envía el comando `c` y presiona Enter

Verás:
```
======================================
INICIANDO CALIBRACIÓN DE SENSORES
======================================
Mueva el robot sobre blanco y negro durante 8 segundos...
Calibrando... 7 segundos restantes
```

#### Paso 2: Movimiento Durante Calibración (CRÍTICO)

Durante los 8 segundos, debes **mover manualmente** el robot asegurando que:

**Estrategia de movimiento:**
```
Segundos 0-4: FASE NEGRO
  → Mueve el robot sobre la LÍNEA NEGRA
  → Asegúrate de que LOS 5 SENSORES pasen sobre el negro
  → Movimientos lentos de lado a lado

Segundos 4-8: FASE BLANCO
  → Mueve el robot sobre la SUPERFICIE BLANCA
  → Asegúrate de que LOS 5 SENSORES pasen sobre el blanco
  → Movimientos lentos de lado a lado
```

**Movimiento correcto:**
```
    [S1][S2][S3][S4][S5]
         Robot
           ↓
    ═══════════════════  ← Línea negra
    ░░░░░░░░░░░░░░░░░░░  ← Fondo blanco

    Mover: ← → ← → ← →
    Asegurar que TODOS los sensores vean ambas superficies
```

> ⚠️ **IMPORTANTE:** Si algún sensor NO ve el negro o el blanco durante la calibración, tendrá un rango muy pequeño y causará errores.

#### Paso 3: Verificar Resultados

Al terminar, verás los valores calibrados:

```
======================================
CALIBRACIÓN COMPLETADA
======================================
VALORES DE CALIBRACIÓN

Sensor | MIN   | MAX   | Rango | Estado
-------|-------|-------|-------|--------
S1 (6) |  120  | 2050  | 1930  | ✓ EXCELENTE
S2 (5) |  105  | 2100  | 1995  | ✓ EXCELENTE
S3 (4) |  135  | 1980  | 1845  | ✓ BUENO
S4 (8) |  118  | 2075  | 1957  | ✓ EXCELENTE
S5 (7) |  142  | 2020  | 1878  | ✓ BUENO
```

**Interpretación:**
- ✅ **Rango > 1500:** Excelente calibración
- ⚠️ **Rango 1000-1500:** Aceptable, considera recalibrar
- ❌ **Rango < 1000:** Mala calibración, DEBE recalibrarse

---

## 4. Verificación de Calibración

### 4.1 Test Estático

Después de calibrar, verifica manualmente:

**Test 1: Línea al centro**
```bash
# Envía el comando
ts
```

Coloca el robot con **sensor 3 (centro) sobre la línea negra**:
```
Valores esperados:
S1: Bajo (~100-300)   [sobre blanco]
S2: Bajo (~100-300)   [sobre blanco]
S3: Alto (~1800-2200) [sobre negro] ← ¡Centro!
S4: Bajo (~100-300)   [sobre blanco]
S5: Bajo (~100-300)   [sobre blanco]

Error calculado: ~0 (centrado)
```

**Test 2: Línea a la izquierda**

Mueve el robot para que la línea quede bajo **sensor 1**:
```
Valores esperados:
S1: Alto (~1800-2200) [sobre negro] ← ¡Izquierda!
S2-S5: Bajo (~100-300) [sobre blanco]

Error calculado: ~-200 (muy a la izquierda)
```

**Test 3: Línea a la derecha**

Mueve el robot para que la línea quede bajo **sensor 5**:
```
Valores esperados:
S1-S4: Bajo (~100-300) [sobre blanco]
S5: Alto (~1800-2200) [sobre negro] ← ¡Derecha!

Error calculado: ~+200 (muy a la derecha)
```

### 4.2 Test Dinámico

Envía el comando `p` para ver el cálculo de posición en tiempo real:
```bash
# Comando
p
```

Mueve lentamente el robot sobre la línea. Deberías ver el error cambiar suavemente de -200 a +200 según la posición.

---

## 5. Problemas Comunes

| Síntoma | Causa Probable | Solución |
|---------|----------------|----------|
| **Rango < 1000 en todos los sensores** | Poca altura de sensores o mala superficie | Acerca sensores a 3-5mm. Usa superficie más mate |
| **Rango < 1000 en UN sensor** | Ese sensor no vio bien el negro/blanco durante calibración | Recalibra asegurando que TODOS vean ambas superficies |
| **Sensor siempre lee 0** | Pin desconectado o sensor sin VCC | Verifica conexión VCC, GND, OUT del sensor |
| **Sensor siempre lee 4095** | Sensor saturado (muy cerca) o mal configurado | Aleja sensor a 5mm. Verifica que NO haya `pinMode()` |
| **Valores muy inestables** | Interferencia o cables largos | Usa cables cortos y trenzados. Aleja de motores |
| **Robot oscila después de calibrar** | Problema de PID, NO de calibración | Ver [tuning_pid.md](tuning_pid.md) |
| **Se sale en curvas** | Velocidad muy alta o PID bajo | Reduce velocidad con `v 100`, luego ajusta PID |

---

## 6. Herramientas de Diagnóstico

### 6.1 Usando `test_pines_adc.ino`

Herramienta para verificar lecturas ADC sin calibración:

```bash
# Sube el test
pio run -t upload --environment test_pines_adc

# Verás salida como:
IZQ+2(G6): 105 | IZQ+1(G5): 112 | CENTRO(G4): 108 | DER+1(G8): 120 | DER+2(G7): 118 |
```

**Sobre blanco:** Todos los valores deben estar entre 80-300
**Sobre negro:** Todos los valores deben estar entre 1700-2300

### 6.2 Usando `test_sensores.ino`

Herramienta avanzada con múltiples comandos:

```bash
# Comandos útiles:
l  - Lectura continua de valores RAW
v  - Ver valores detallados con min/max
c  - Calibración guiada
b  - Detección binaria visual (█ = negro, ░ = blanco)
s  - Test de sensibilidad (detecta defectuosos)
e  - Test de estabilidad (mide ruido)
```

### 6.3 Comandos del Programa Principal

```bash
# Desde main.cpp:
c     - Iniciar calibración automática (8s)
ts    - Test sensores en tiempo real
s     - Ver estado completo del sistema
r     - Reset calibración
```

---

## 📊 Checklist Final

Antes de navegar, verifica:

- [ ] Todos los 5 sensores tienen rango > 1500
- [ ] Test estático: error = 0 cuando línea está al centro
- [ ] Test estático: error negativo cuando línea a la izquierda
- [ ] Test estático: error positivo cuando línea a la derecha
- [ ] No hay sensores con lectura constante 0 o 4095
- [ ] Iluminación es estable (no luz solar directa)

---

**Siguiente paso:** [tuning_pid.md](tuning_pid.md) para optimizar el control del robot.

**Última actualización:** 2025-01-06
