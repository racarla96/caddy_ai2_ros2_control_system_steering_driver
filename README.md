# Caddy AI2 ROS2 Control System Steering Driver

Plugin de tipo **Controller** (`controller_interface::ControllerInterface`) para ROS2 Control que gestiona el sistema de dirección del robot Caddy AI2 mediante CANopen sobre SocketCAN.

## Descripción

Este paquete implementa un **Controller** de ROS2 Control que sustituye a la antigua arquitectura basada en Hardware Interface. El controlador gestiona directamente la comunicación CAN/CANopen sin necesidad de un hardware interface separado:

- **Motor driver** con protocolo CANopen (CiA 402)
- **Encoder absoluto** externo con protocolo CANopen
- **Comunicación CAN** mediante SocketCAN (Linux)
- **Interfaz ROS2**: recibe posición objetivo vía tópico y publica posición medida

## Arquitectura

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Control Manager                      │
│                    (frecuencia configurable)                  │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│           SteeringDriverController (Controller)              │
│                                                              │
│  ~/reference (std_msgs/Float64) ──► posición objetivo [rad] │
│  ~/state     (std_msgs/Float64) ◄── posición encoder [rad]  │
│                                                              │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  • Gestión de frecuencias (ratio, multiplicidades)   │   │
│  │  • Conversión radianes ↔ encoder counts              │   │
│  │  │  Offsets de lectura/escritura                     │   │
│  └──────────────────────────────────────────────────────┘   │
│                         │                                    │
│                         ▼                                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │           SteeringController                         │   │
│  │  ┌────────────────┐  ┌────────────────┐             │   │
│  │  │  MotorDriver   │  │ EncoderDriver  │             │   │
│  │  │  (CiA 402)     │  │  (absoluto)    │             │   │
│  │  │  • PDO/SDO     │  │  • PDO/SDO     │             │   │
│  │  └────────────────┘  └────────────────┘             │   │
│  └──────────────────────────────────────────────────────┘   │
│                         │                                    │
│                         ▼                                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │         SocketCANInterface                           │   │
│  │  • Socket CAN RAW                                    │   │
│  │  • Epoll / Non-blocking I/O                         │   │
│  └──────────────────────────────────────────────────────┘   │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
                  ┌─────────────┐
                  │   CAN Bus   │
                  │  (can0/vcan)│
                  └─────────────┘
                         │
        ┌────────────────┴────────────────┐
        ▼                                 ▼
  ┌──────────┐                     ┌──────────┐
  │  Motor   │                     │ Encoder  │
  │ CANopen  │                     │ CANopen  │
  └──────────┘                     └──────────┘
```

## Componentes

### 1. SteeringDriverController
Controller principal que implementa `controller_interface::ControllerInterface`.

**Lifecycle:**
- `on_init()`: Crea el `ParamListener` para los parámetros generados.
- `on_configure()`: Instancia `SteeringController`, calcula multiplicidades efectivas, crea subscriber y publisher.
- `on_activate()`: Llama a `SteeringController::init()` (inicializa CAN y CANopen), resetea contadores con offsets.
- `update()`: Ejecuta la lógica de lectura/escritura con multiplicidades. Llama a `step()`, lee encoder, publica posición, envía consigna.
- `on_deactivate()`: Llama a `SteeringController::shutdown()` (deshabilita motor de forma segura).
- `on_cleanup()`: Destruye el controlador y los recursos ROS2.

**Interfaz ROS2:**

| Tópico | Dirección | Tipo | Descripción |
|--------|-----------|------|-------------|
| `<ns>/reference` | entrada | `std_msgs/Float64` | Posición objetivo en radianes |
| `<ns>/state` | salida | `std_msgs/Float64` | Posición medida del encoder en radianes |

### 2. SteeringController
Coordinador de alto nivel entre motor y encoder CANopen.

- `init()`: Inicializa SocketCAN, arranca los nodos CANopen en OPERATIONAL.
- `step()`: Procesa frames CAN entrantes, actualiza máquinas de estado, envía SYNC y consigna.
- `setTargetSteeringPosition(counts)`: Establece posición objetivo en counts.
- `getAbsoluteEncoderPosition()`: Lee posición del encoder absoluto en counts.
- `shutdown()`: Deshabilita el motor de forma segura.

### 3. MotorDriver (CiA 402)
Driver para motor con protocolo CANopen CiA 402. Gestiona la máquina de estados y la comunicación PDO/SDO.

### 4. EncoderDriver
Driver para encoder absoluto externo. Lee posición vía TPDOs CANopen.

### 5. SocketCANInterface
Interfaz de bajo nivel para comunicación CAN (socket RAW, epoll, non-blocking I/O).

### 6. CANOpenDriver
Clase base abstracta con funcionalidades comunes: gestión NMT, SDO/PDO, NodeGuard, timeouts.

## Parámetros

Todos los parámetros se declaran mediante `generate_parameter_library` y son `read_only` (fijados en configuración):

| Parámetro | Tipo | Por defecto | Descripción |
|-----------|------|-------------|-------------|
| `interface_name` | string | — | Nombre de la interfaz SocketCAN (ej. `can0`) |
| `controller_manager_frequency_hz` | double | 100.0 | Frecuencia de actualización del controller manager (Hz) |
| `hardware_sample_frequency_hz` | double | 500.0 | Frecuencia de muestreo del bus CAN (Hz) |
| `read_multiplicity` | int | 1 | Ciclos del CM entre lecturas del encoder |
| `write_multiplicity` | int | 10 | Ciclos del CM entre escrituras al motor |
| `read_offset` | int | 0 | Desfase inicial del contador de lectura (ciclos) |
| `write_offset` | int | 1 | Desfase inicial del contador de escritura (ciclos) |
| `motor_node_id` | int | 1 | Node ID CANopen del motor (1–127) |
| `encoder_node_id` | int | 127 | Node ID CANopen del encoder (1–127) |
| `counts_per_radian` | double | 1.0 | Factor de conversión encoder counts/rad |

### Ejemplo de configuración (YAML del controller manager)

```yaml
steering_driver_controller:
  ros__parameters:
    interface_name: can_steer_drv
    controller_manager_frequency_hz: 100.0
    hardware_sample_frequency_hz: 500.0
    read_multiplicity: 1
    write_multiplicity: 10
    read_offset: 0
    write_offset: 1
    motor_node_id: 1
    encoder_node_id: 127
    counts_per_radian: 100330.0
```

### Plugin (controller_manager config)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    steering_driver_controller:
      type: caddy_ai2_ros2_control_system_steering_driver/SteeringDriverController
```

## Cálculo de Frecuencias y Multiplicidades

```
frequency_ratio             = controller_manager_frequency_hz / hardware_sample_frequency_hz
effective_read_multiplicity  = read_multiplicity  × frequency_ratio
effective_write_multiplicity = write_multiplicity × frequency_ratio

Frecuencia real lectura  = controller_manager_frequency_hz / effective_read_multiplicity
Frecuencia real escritura = controller_manager_frequency_hz / effective_write_multiplicity
```

**Ejemplo** (CM a 100 Hz, hardware a 500 Hz):
```
frequency_ratio = 100 / 500 = 0.2  →  se fuerza a 1 (mínimo)
effective_read_multiplicity  = 1  × 1 = 1   → lectura  a 100 Hz
effective_write_multiplicity = 10 × 1 = 10  → escritura a  10 Hz
```

Los **offsets** permiten desfasar lectura y escritura dentro del mismo periodo:
```
Ciclo:  0   1   2   3  ...  9  10  ...
Read:   R   ─   ─   ─  ...  ─   R  ...   (offset=0, mult=1)
Write:  ─   W   ─   ─  ...  ─   ─  ...   (offset=1, mult=10)
```

## Instalación y compilación

### Dependencias

```bash
# ROS2 Jazzy
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# CAN tools
sudo apt install can-utils
```

### Compilar

```bash
cd ~/ws_ros2_caddy_dev
colcon build --packages-select caddy_ai2_ros2_common caddy_ai2_ros2_control_system_steering_driver
source install/setup.bash
```

## Configuración del bus CAN

### CAN virtual (desarrollo/simulación)

```bash
sudo ./scripts/setup_vcan_steer_drv.sh
# o manualmente:
sudo modprobe vcan
sudo ip link add dev vcan_steer_drv type vcan
sudo ip link set up vcan_steer_drv
```

### CAN físico (hardware real)

```bash
sudo ./scripts/setup_can_steer_drv.sh
# o manualmente:
sudo ip link set can_steer_drv type can bitrate 500000
sudo ip link set up can_steer_drv
```

## Uso

### Enviar posición objetivo

```bash
ros2 topic pub /steering_driver_controller/reference std_msgs/msg/Float64 "data: 0.3"
```

### Leer posición medida

```bash
ros2 topic echo /steering_driver_controller/state
```

### Estado del controller

```bash
ros2 control list_controllers
```

## Estructura del proyecto

```
caddy_ai2_ros2_control_system_steering_driver/
├── include/
│   └── caddy_ai2_ros2_control_system_steering_driver/
│       ├── steering_driver_controller.hpp   # Controller (nuevo)
│       ├── steering_controller.hpp          # Coordinador CAN
│       ├── motor_driver.hpp                 # Driver motor CANopen
│       ├── encoder_driver.hpp               # Driver encoder CANopen
│       └── canopen_driver.hpp               # Clase base CANopen
├── src/
│   ├── steering_driver_controller.cpp       # Controller (nuevo)
│   ├── steering_driver_controller_parameters.yaml
│   ├── steering_controller.cpp
│   ├── motor_driver.cpp
│   ├── encoder_driver.cpp
│   └── canopen_driver.cpp
├── scripts/
│   ├── setup_can_steer_drv.sh              # Configuración CAN físico
│   └── setup_vcan_steer_drv.sh             # Configuración CAN virtual
├── plugin_description.xml                   # Registro del plugin
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Troubleshooting

### Interfaz CAN no encontrada

```bash
ip link show   # listar interfaces disponibles
```

### Motor no responde

1. Verificar Node ID y bitrate del bus.
2. Comprobar cableado CAN (CANH, CANL, GND) y terminación (120 Ω en ambos extremos).
3. Monitorear tráfico: `candump can_steer_drv`

### Encoder no válido

1. Verificar alimentación y Node ID del encoder.
2. Comprobar configuración de TPDOs en el encoder.

### Limpiar y recompilar

```bash
cd ~/ws_ros2_caddy_dev
rm -rf build/caddy_ai2_ros2_control_system_steering_driver \
       install/caddy_ai2_ros2_control_system_steering_driver
colcon build --packages-select caddy_ai2_ros2_control_system_steering_driver
```

## Referencias

- [ROS2 Control — Writing a Controller](https://control.ros.org/jazzy/doc/ros2_control/controller_interface/doc/writing_new_controller.html)
- [CANopen CiA 402 Specification](https://www.can-cia.org/can-knowledge/canopen/cia402/)
- [SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)

## Autores

- **Desarrollador Principal**: Rafael Carbonell Lázaro (racarla96)
- **Proyecto**: Caddy AI2 – Proyecto CERVAREC

## Licencia

Copyright (c) 2025, Rafael Carbonell Lázaro (racarla96)

Distribuido bajo la licencia **Creative Commons Attribution 4.0 International (CC BY 4.0)**.
https://creativecommons.org/licenses/by/4.0/
