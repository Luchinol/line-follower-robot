# Diagrama de Funciones - Carrito Seguidor de Línea ESP32-S3 (Config. Simplificada)

## Diagrama de Flujo Principal

```mermaid
flowchart TD
    Start([Inicio]) --> Setup[setup<br/>Inicialización del Sistema]
    Setup --> InitNVS[configNVS.inicializar<br/>Cargar config. guardada]
    InitNVS --> InitButtons[inicializarBotones<br/>Configurar interrupciones]
    InitNVS --> InitSensors[sensores.inicializar<br/>Configurar pines ADC]
    InitSensors --> InitMotors[motores.inicializar<br/>Configurar PWM y GPIO]
    InitMotors --> InitPID[pid.inicializar<br/>Reset variables PID]
    InitPID --> StartCalib[sensores.iniciarCalibracion<br/>Iniciar calibración de 8s]
    StartCalib --> Loop[loop<br/>Ciclo Principal]

    Loop --> ProcFlags[procesarBanderas<br/>Leer botones físicos]
    ProcFlags --> ProcCmd[procesarComandosSerial<br/>Leer comandos USB]
    ProcCmd --> StateMachine{Máquina de Estados}

    StateMachine -->|CALIBRANDO| StateCalib[estadoCalibrar]
    StateMachine -->|SIGUIENDO_LINEA| StateFollow[estadoSeguirLinea]
    StateMachine -->|PERDIDA_LINEA| StateLost[estadoPerdidaLinea]
    StateMachine -->|BUSCANDO_LINEA| StateSearch[estadoBuscarLinea]
    StateMachine -->|PAUSADO| StatePaused[estadoPausado]
    StateMachine -->|DETENIDO| StateStop[estadoDetenido]

    StateCalib --> Delay[delay 10ms]
    StateFollow --> Delay
    StateLost --> Delay
    StateSearch --> Delay
    StatePaused --> Delay
    StateStop --> Delay

    Delay --> Loop

    style Start fill:#90EE90
    style Loop fill:#87CEEB
    style StateMachine fill:#FFD700
    style StateFollow fill:#FF6347
```

## Diagrama de Estado: SIGUIENDO_LINEA (Lógica Actual)

Este diagrama refleja la lógica simplificada con 3 sensores. No hay "fusión de sensores" ni "PID adaptativo" basado en curvatura.

```mermaid
flowchart TD
    Start([estadoSeguirLinea]) --> ReadSensors[sensores.leer<br/>Leer 3 sensores ADC]
    ReadSensors --> Process[sensores.procesar<br/>Calibrar valores 0-100<br/>Calcular posición ponderada]

    Process --> CheckLine{sensores.lineaDetectada<br/>¿Línea visible?}
    CheckLine -->|NO| Lost[tiempoPerdidaLinea = millis<br/>cambiarEstado(PERDIDA_LINEA)]
    CheckLine -->|SÍ| GetError[error = sensores.obtenerErrorFusionado<br/><b>(Devuelve error de los 3 sensores)</b>]

    GetError --> CalcPID[correccion = pid.calcular(error)<br/>P = Kp × error<br/>I = Ki × Σerror<br/>D = Kd × Δerror/Δt]

    CalcPID --> CalcVel[velIzq = velocidadBase - correccion<br/>velDer = velocidadBase + correccion]
    CalcVel --> ApplyMotors[motores.diferencial<br/>setMotorIzquierdo<br/>setMotorDerecho]
    ApplyMotors --> End([Return])

    Lost --> End

    style Start fill:#90EE90
    style CheckLine fill:#FFD700
    style GetError fill:#87CEEB
    style End fill:#90EE90
```

## Diagrama de Clases (Simplificado)

### `SensoresIR`
Gestiona los 3 sensores.
```mermaid
classDiagram
    class SensoresIR {
        -uint8_t pinesCercanos[3]
        -uint16_t valoresCercanos[3]
        -uint8_t valoresCalibrados[3]
        -uint16_t minCercanos[3]
        -uint16_t maxCercanos[3]
        -bool calibrado
        -int16_t ultimaPosicionCercano

        +inicializar() void
        +iniciarCalibracion() void
        +actualizarCalibracion() bool
        +leer() void
        +procesar() void
        +obtenerErrorFusionado() int16_t
        +lineaDetectada() bool
        -calcularPosicion(valores, num) int16_t
    }
```

### `ControladorPID`
Implementa el algoritmo PID con un único set de parámetros.
```mermaid
classDiagram
    class ControladorPID {
        -float Kp, Ki, Kd
        -float errorAnterior
        -float integral
        -float derivadaFiltrada

        +inicializar() void
        +reset() void
        +calcular(error) float
        +setParametros(kp, ki, kd) void
        -aplicarAntiWindup() void
    }
```

## Diagrama de Secuencia: Ciclo de Seguimiento Simplificado

```mermaid
sequenceDiagram
    participant M as main.cpp
    participant S as SensoresIR
    participant P as ControladorPID
    participant Mo as ControlMotores

    M->>M: loop() se ejecuta
    M->>S: leer() (lee 3 sensores)
    activate S
    S-->>M: valores[3] ADC
    deactivate S

    M->>S: procesar()
    activate S
    S->>S: calcularPosicion() con 3 sensores
    S-->>M: error calculado
    deactivate S

    M->>S: lineaDetectada()
    S-->>M: true/false

    alt Línea detectada
        M->>S: obtenerErrorFusionado()
        S-->>M: error (de los 3 sensores)

        M->>P: calcular(error)
        activate P
        P->>P: salida = (Kp*E) + (Ki*∫E) + (Kd*dE/dt)
        P-->>M: corrección
        deactivate P

        M->>M: velIzq = base - corrección<br/>velDer = base + corrección

        M->>Mo: diferencial(velIzq, velDer)
        activate Mo
        Mo-->>M: Motores actualizados
        deactivate Mo

    else Línea perdida
        M->>M: cambiarEstado(PERDIDA_LINEA)
    end
```

---
**Última actualización:** 2025-11-05