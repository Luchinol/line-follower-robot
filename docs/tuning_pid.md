> **Nota Importante:** Esta guía explica los conceptos generales del control PID. La versión actual del código **no utiliza el PID adaptativo** (diferentes parámetros para rectas y curvas) porque se ha simplificado a una configuración de 3 sensores. Por lo tanto, solo necesitas ajustar un único conjunto de parámetros PID (los que corresponden a `PID_RECTA` en `config.h`). Ignora las secciones sobre "PID Adaptativo" y "Fusión de Sensores", ya que no aplican a la lógica actual.

# Guía Avanzada de Ajuste PID

El ajuste del controlador PID es un arte y una ciencia. Esta guía te enseñará cómo optimizar los parámetros para que tu robot siga la línea de forma suave y rápida.

## 📋 Tabla de Contenidos

1. [Fundamentos del PID](#fundamentos-del-pid)
2. [Metodología de Ajuste](#metodología-de-ajuste)
3. [Ajuste Paso a Paso](#ajuste-paso-a-paso)
4. [Casos de Estudio y Troubleshooting](#casos-de-estudio-y-troubleshooting)

---

## 1. Fundamentos del PID

### 1.1 La Ecuación PID

La corrección que aplica el robot se calcula así:

`Corrección = (Kp * Error) + (Ki * Suma_de_Errores) + (Kd * Cambio_en_el_Error)`

- **`Error`**: Qué tan lejos está el robot de la línea.
- **`Kp` (Proporcional)**: El trabajador principal. Reacciona al error **actual**. Un `Kp` alto da una respuesta fuerte y rápida, pero puede causar inestabilidad (oscilaciones).
- **`Ki` (Integral)**: El perfeccionista. Corrige errores pequeños y persistentes que se acumulan con el tiempo. Ayuda a que el robot se centre perfectamente en la línea.
- **`Kd` (Derivativo)**: El predictor. Actúa sobre la **velocidad de cambio** del error. Frena las reacciones bruscas de `Kp`, amortiguando las oscilaciones y suavizando el movimiento.

### 1.2 Visualización del Comportamiento

- **Solo `Kp`**: El robot probablemente oscilará de lado a lado sobre la línea.
- **`Kp` + `Kd`**: Las oscilaciones se reducirán y el movimiento será más suave.
- **`Kp` + `Ki` + `Kd`**: El robot no solo seguirá la línea suavemente, sino que también se centrará perfectamente sin desviaciones a largo plazo.

---

## 2. Metodología de Ajuste

La forma más sencilla y efectiva es ajustar los parámetros en orden: primero `Kp`, luego `Kd` y finalmente `Ki`.

### Herramienta Clave: Ajuste en Vivo por Serial

No necesitas recompilar el código para cada cambio. Usa el monitor serial:

```bash
# Envía este comando para cambiar los parámetros en tiempo real
p 1.5 0.05 0.8

# Para guardar los nuevos valores permanentemente (incluso después de apagar)
save
```

---

## 3. Ajuste Paso a Paso

### 3.1 Configuración Inicial

1.  Asegúrate de que la **calibración de sensores es perfecta**. Un mal ajuste de PID no puede compensar una mala calibración.
2.  Establece una velocidad base moderada para las pruebas. Envía `v 140`.
3.  Desactiva `Ki` y `Kd` para empezar. Envía `p [tu_kp_inicial] 0 0` (un buen `Kp` para empezar es `1.0`).

### 3.2 PASO 1: Ajustar `Kp` (el Proporcional)

**Objetivo**: Que el robot siga la línea, aunque oscile un poco.

1.  Coloca el robot en una sección recta de la pista.
2.  Empieza con un `Kp` bajo (ej: `p 1.0 0 0`).
3.  Observa el comportamiento y ajusta:
    - **Si el robot no reacciona y se va de la línea**: `Kp` es muy bajo. Auméntalo (ej: `p 1.5 0 0`).
    - **Si el robot sigue la línea pero de forma lenta o "perezosa"**: Aumenta `Kp` en pequeños incrementos.
    - **Si el robot oscila violentamente (zigzaguea)**: `Kp` es muy alto. Redúcelo.

> **Meta del Paso 1**: Encuentra un valor de `Kp` que haga que el robot siga la línea de forma estable, incluso si tiene una oscilación pequeña y constante. Este es tu "Kp crítico".

### 3.3 PASO 2: Agregar `Kd` (el Derivativo)

**Objetivo**: Amortiguar y eliminar las oscilaciones causadas por `Kp`.

1.  Mantén el `Kp` que encontraste en el paso anterior.
2.  Empieza a añadir `Kd`. Una buena regla general es empezar con `Kd` siendo la mitad de `Kp`.
    (ej: si tu `Kp` es `2.0`, prueba `p 2.0 0 1.0`).
3.  Observa y ajusta:
    - **Si el robot todavía oscila**: Aumenta `Kd`.
    - **Si el robot se vuelve lento para reaccionar o parece "pesado"**: `Kd` es muy alto. Redúcelo.

> **Meta del Paso 2**: Encontrar un valor de `Kd` que elimine casi por completo las oscilaciones, resultando en un seguimiento suave y directo en las rectas.

### 3.4 PASO 3: Agregar `Ki` (el Integral)

**Objetivo**: Eliminar cualquier error pequeño y persistente (offset).

**Solo necesitas `Ki` si notas que el robot sigue la línea pero consistentemente un poco desviado hacia un lado.** Si ya se centra bien, puedes dejar `Ki` en `0`.

1.  Mantén `Kp` y `Kd`.
2.  Añade un valor muy pequeño de `Ki`. Regla general: `Ki` debe ser mucho menor que `Kp` (ej: si `Kp` es `2.0`, prueba `p 2.0 [tu_ki] 1.0` con `[tu_ki]` siendo `0.01`).
3.  Observa en una recta larga:
    - **Si el robot ahora se centra perfectamente**: ¡Éxito!
    - **Si el robot empieza a tener oscilaciones lentas y amplias**: `Ki` es muy alto. Redúcelo.

> **Peligro de `Ki` (Wind-up)**: Un valor de `Ki` muy alto puede hacer que el error acumulado "explote" en curvas largas, haciendo que el robot se descontrole. Por eso se usa con moderación.

---

## 4. Casos de Estudio y Troubleshooting

| Síntoma | Diagnóstico Probable | Solución Rápida |
|---|---|---|
| **Oscila mucho (zig-zag rápido)** | `Kp` es demasiado alto. | Reduce `Kp` en un 20-30%. |
| **Sigue la línea, pero no en el centro** | Falta de término `Ki`. | Añade un `Ki` pequeño (ej: 0.01-0.05). |
| **Se sale en curvas cerradas** | `Kp` es bajo o la velocidad es muy alta. | Aumenta `Kp` o reduce la velocidad (`v [nueva_vel]`). |
| **Movimiento "tembloroso" o errático** | `Kd` es muy alto o hay ruido en los sensores. | Reduce `Kd`. Asegura una buena calibración y luz estable. |
| **Respuesta muy lenta, perezosa** | `Kp` es demasiado bajo. | Aumenta `Kp`. |

### Valores de Referencia (para empezar)

Los valores por defecto en tu `config.h` son un excelente punto de partida:

- **`Kp`**: 2.5
- **`Ki`**: 0.03
- **`Kd`**: 0.5

Comienza con estos valores (`p 2.5 0.03 0.5`) y ajústalos siguiendo la metodología descrita.

---

**Última actualización:** 2025-11-05