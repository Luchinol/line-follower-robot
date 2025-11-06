# Carrito Seguidor de Línea ESP32-S3

Sistema de seguimiento de línea autónomo con **ESP32-S3**, 3 sensores IR y control PID. El proyecto ha sido simplificado para usar una configuración robusta y fácil de calibrar.

## 🎯 Hardware Utilizado

| Componente | Modelo | Cantidad |
|------------|--------|----------|
| **Microcontrolador** | ESP32-S3 WROOM (FREENOVE) | 1 |
| **Sensores IR** | HW-511 (o similar) individuales | 3 |
| **Puente H** | L298N | 1 |
| **Motores DC** | Con reductora 1:48 | 2 |
| **Batería** | LiPo 2S 7.4V o 6xAA | 1 |

> **Nota**: El código está configurado actualmente para 3 sensores, pero la estructura soporta configuraciones más complejas.

## ⚡ Inicio Rápido

### 1. Conexiones (Configuración Actual)

```
MOTORES (ESP32-S3 → L298N):
  GPIO 12 → ENA    // PWM Motor Derecho
  GPIO 11 → IN1
  GPIO 18 → IN2

  GPIO 13 → ENB    // PWM Motor Izquierdo
  GPIO 14 → IN3
  GPIO 21 → IN4

SENSORES (3x HW-511 → ESP32-S3):
  GPIO 3 → Sensor Izquierdo (A0)
  GPIO 4 → Sensor Central   (A0)
  GPIO 5 → Sensor Derecho  (A0)

ALIMENTACIÓN:
  Batería 7.4V → L298N (+12V)
  L298N 5V → Sensores VCC
  L298N 5V → ESP32 VIN (Opcional si se usa USB)
  GND → Conectar todos los GND juntos.
```

### 2. Programar

```bash
# Usando PlatformIO (Recomendado)
pio run -t upload && pio device monitor

# Usando Arduino IDE
# Placa: "ESP32S3 Dev Module"
# USB CDC On Boot: "Enabled"
```

### 3. Calibrar Sensores

Este es el paso más importante.

```
1. Abrir el Monitor Serial (baudrate: 115200).
2. Enviar el comando 'c' para iniciar la calibración.
3. Durante 8 segundos, mover el robot manualmente para que los 3 sensores pasen varias veces sobre la LÍNEA NEGRA y el FONDO BLANCO.
4. El sistema aprenderá los valores mínimos y máximos y estará listo.
```

### 4. ¡A rodar!

El robot iniciará el seguimiento de línea automáticamente después de la calibración. Usa los comandos seriales para pausar, ajustar o reiniciar.

## 🎮 Comandos Seriales

| Comando | Descripción |
|---|---|
| `c` | **Iniciar calibración** de sensores. |
| `s` | Ver **estado** del sistema (estado, velocidad, PID). |
| `r` | **Reiniciar** el seguimiento de línea. |
| `d` | Ejecutar un **diagnóstico** de hardware. |
| `p [Kp] [Ki] [Kd]` | Ajustar los parámetros **PID** en tiempo real. | 
| `v [vel]` | Cambiar la **velocidad base** del robot (0-255). |
| `save` | **Guardar** la configuración actual de PID y velocidad en la memoria Flash. |
| `h` o `?` | Mostrar la lista completa de **ayuda**. |
| `0` / `1` | Atajos para **pausar** y **reanudar**.|

**Ejemplo de ajuste:**
`p 2.5 0.03 0.5` - Ajusta los parámetros PID a los valores por defecto para una recta.
`v 140` - Establece la velocidad base a 140.
`save` - Guarda estos nuevos valores para que se usen la próxima vez que enciendas el robot.

## 🔧 Características Principales

### Control PID
El corazón del robot es un controlador PID que calcula la corrección necesaria para mantenerse en la línea. 
- **`Kp` (Proporcional):** Reacciona al error actual. Un `Kp` alto da una respuesta rápida pero puede causar oscilaciones.
- **`Ki` (Integral):** Corrige errores pequeños y persistentes a lo largo del tiempo.
- **`Kd` (Derivativo):** Amortigua la respuesta y previene el exceso de corrección (overshoot).

Con la configuración actual de 3 sensores, el sistema **no usa el PID adaptativo** que se menciona en los comentarios del código, sino que utiliza un único conjunto de parámetros PID.

### Lógica de Sensores
- Se leen 3 sensores analógicos.
- Durante la calibración, se registran los valores mínimos (blanco) y máximos (negro) para cada sensor.
- Se calcula una posición ponderada de la línea, dando un valor de error entre -100 (muy a la izquierda) y +100 (muy a la derecha).
- Este error es la entrada para el controlador PID.

### Persistencia de Configuración (NVS)
Gracias al módulo `nvs_config.h`, puedes ajustar los parámetros PID y la velocidad base a través del monitor serial y guardarlos. No se perderán al apagar el robot.

## 🐛 Troubleshooting Rápido

| Problema | Solución Sugerida |
|---|---|
| **Oscila mucho en la recta** | El parámetro `Kp` es muy alto. Redúcelo con `p [nuevo_kp] [ki] [kd]` y guarda. |
| **Se sale en las curvas** | La velocidad es muy alta o `Kp` es muy bajo. Prueba bajar la velocidad con `v [nueva_vel]` o subir `Kp`. |
| **Se mueve erráticamente** | La calibración falló. Recalibra (`c`) asegurándote de que todos los sensores vean bien el blanco y el negro. |
| **Un motor gira más lento**| Ajusta el `FACTOR_MOTOR_IZQUIERDO` o `FACTOR_MOTOR_DERECHO` en `src/config.h`. |
| **No responde a comandos** | Verifica que el baudrate del monitor serial sea `115200` y que la línea termine en `NL & CR`. |

## 📂 Estructura del Código

```
CarritoSeguidor/
├── README.md                # Esta guía
├── platformio.ini           # Configuración de PlatformIO
│
├── src/                     # Código fuente principal
│   ├── main.cpp             # Lógica principal, máquina de estados y comandos
│   ├── config.h             # ⭐ TODOS los pines y parámetros importantes
│   ├── sensores.h           # Lógica para leer y calibrar los 3 sensores
│   ├── motores.h            # Control de bajo nivel de los motores
│   ├── control_pid.h        # Implementación del controlador PID
│   └── nvs_config.h         # Lógica para guardar/cargar configuración
│
└── docs/                    # Documentación detallada
    ├── calibracion.md       # Guías de calibración
    └── tuning_pid.md        # Guías para el ajuste del PID
```

## 📄 Licencia

MIT License - Ver [LICENSE](LICENSE) para más detalles.