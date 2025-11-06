> **Nota Importante:** Esta guía fue escrita para una versión del código que usaba 10 sensores en dos arrays. La versión actual se ha simplificado a **3 sensores**. Los conceptos de calibración (encontrar mínimos y máximos) son exactamente los mismos, pero ignora las referencias a "array lejano" y "array cercano". El proceso de calibración actual funciona con los 3 sensores centrales.

# Guía Completa de Calibración de Sensores IR

La calibración es el paso **MÁS IMPORTANTE** para que tu robot siga la línea correctamente. Una mala calibración hará que el robot se comporte erráticamente sin importar qué tan bien ajustes el PID.

## 📋 Tabla de Contenidos

1. [¿Por qué calibrar?](#por-qué-calibrar)
2. [Preparación](#preparación)
3. [Proceso de Calibración](#proceso-de-calibración)
4. [Verificación de Calibración](#verificación-de-calibración)
5. [Problemas Comunes](#problemas-comunes)

---

## 1. ¿Por qué calibrar?

### 1.1 El Problema

Cada sensor IR es ligeramente diferente. Sin calibración, el robot no puede interpretar las lecturas de forma fiable.

```
// Ejemplo sin calibración:
Sensor Izquierdo sobre BLANCO lee: 300
Sensor Central sobre BLANCO lee: 450

Sensor Izquierdo sobre NEGRO lee: 2800
Sensor Central sobre NEGRO lee: 3200

Problema: ¡El robot no tiene una referencia clara de qué es blanco y qué es negro!
```

### 1.2 La Solución

La calibración le "enseña" al robot el rango de lecturas para cada sensor, normalizando los valores (en el código, a una escala de 0 a 1000) para que puedan ser comparados de forma consistente.

```cpp
// Lógica de la calibración:
Para cada sensor:
1. Encontrar el valor MÍNIMO (sobre blanco) durante el proceso.
2. Encontrar el valor MÁXIMO (sobre negro) durante el proceso.

// En tiempo real, el valor leído se mapea usando esos mínimos y máximos.
valor_calibrado = map(valor_actual, mínimo_aprendido, máximo_aprendido, 0, 1000)
```

---

## 2. Preparación

### 2.1 Requisitos de la Pista

- **Fondo:** Blanco y MATE. Evita superficies brillantes.
- **Línea:** Negra y MATE. La cinta aislante negra funciona bien. Ancho ideal: 1.5 - 2.5 cm.
- **Iluminación:** Luz artificial uniforme. **Evita la luz solar directa**, ya que interfiere con los sensores IR.

### 2.2 Verificación Pre-Calibración

- **Altura de los sensores:** Entre 2 y 10 mm del suelo. Un buen punto de partida es **5 mm**.
- **Alineación:** Los 3 sensores deben estar a la misma altura y paralelos al suelo.
- **Conexiones:** Asegúrate de que los sensores estén conectados a los GPIO 3, 4 y 5.

---

## 3. Proceso de Calibración

### 3.1 Calibración Automática

#### Paso 1: Iniciar Calibración

1.  Coloca el robot en la pista.
2.  Conecta el ESP32 a tu PC y abre el **Monitor Serial** (baudrate 115200).
3.  Envía el comando `c` y presiona Enter.

Verás un mensaje indicando que la calibración ha comenzado y una cuenta regresiva de 8 segundos.

#### Paso 2: Mover el Robot (Paso Crítico)

Durante los 8 segundos, debes **mover manualmente** el robot sobre la pista de manera que los 3 sensores pasen repetidamente sobre la superficie blanca y la línea negra.

**Movimiento recomendado:**
- **Segundos 0-4:** Mueve el robot lentamente de lado a lado sobre la **línea negra**, asegurándote de que los sensores izquierdo y derecho también pasen por encima de la línea.
- **Segundos 4-8:** Haz lo mismo sobre la **superficie blanca** al lado de la línea.

> **El objetivo es que cada sensor registre su lectura más alta (sobre el negro) y su lectura más baja (sobre el blanco).**

#### Paso 3: Verificar Resultados

Al terminar, el monitor serial mostrará los valores de calibración. Deberías ver algo así:

```
CALIBRACIÓN COMPLETADA
========================================

VALORES DE CALIBRACIÓN:

Sensores Cercanos (5 cm):  <-- (Ignora el nombre, son tus 3 sensores)
Sensor | Min  | Max  | Rango
-------|------|------|-------
  0    | 120  | 2950 | 2830  ✓ BUENO
  1    | 110  | 3100 | 2990  ✓ BUENO
  2    | 135  | 2890 | 2755  ✓ BUENO
```

Un **Rango (Max - Min) grande** (idealmente > 1500) indica una buena calibración y un buen contraste entre la línea y el fondo.

---

## 4. Verificación de Calibración

Después de calibrar, puedes hacer una prueba rápida:

1.  Coloca el robot con el **sensor central justo sobre la línea negra**.
2.  Envía el comando `ts` (test sensores).
3.  La lectura del sensor central debería ser alta (cercana a su `Max`), y las de los sensores izquierdo y derecho deberían ser bajas (cercanas a su `Min`). El `Error` calculado debería ser cercano a `0`.
4.  Mueve el robot para que la línea quede bajo el **sensor izquierdo**.
5.  La lectura del sensor izquierdo debería ser alta y el `Error` debería ser un valor negativo (ej. `-100`).

Si esto funciona, ¡la calibración fue un éxito!

---

## 5. Problemas Comunes

| Síntoma | Causa Probable | Solución Rápida |
|---|---|---|
| **El rango de calibración es muy bajo (< 1000)** | El sensor no vio bien el blanco y el negro, o la pista tiene mal contraste. | Recalibra moviendo más el robot. Usa una superficie más mate. |
| **Un sensor siempre lee 0 o 4095** | Sensor desconectado, mal alimentado o dañado. | Revisa el cableado (VCC, GND, Señal) de ese sensor. |
| **El robot se comporta erráticamente** | La iluminación del ambiente cambió o la calibración fue deficiente. | Recalibra en las condiciones de luz actuales. |
| **Oscila mucho** | Esto suele ser un problema de PID, no de calibración. | Revisa la guía de `tuning_pid.md`. |

---

**Siguiente paso:** [tuning_pid.md](tuning_pid.md) para optimizar el control del robot.

**Última actualización:** 2025-11-05