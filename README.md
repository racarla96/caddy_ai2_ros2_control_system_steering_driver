# Caddy AI2 ROS2 Control System Steering Driver

Sistema de control de dirección para ROS2 Control utilizando CANopen sobre SocketCAN.

## TODOs:
- [ ] Cambiar el código de tipo sistema a tipo actuador

## 📋 Descripción

Este paquete implementa un hardware interface de ROS2 Control para un sistema de dirección basado en:
- **Motor driver** con protocolo CANopen (CiA 402)
- **Encoder absoluto** externo con protocolo CANopen
- **Comunicación CAN** mediante SocketCAN (Linux)

El sistema permite controlar la posición angular de la dirección con alta precisión y frecuencias de actualización configurables.

## 🏗️ Arquitectura

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Control Manager                      │
│                    (500 Hz configurable)                     │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│           SystemSteeringHardware (Hardware Interface)        │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  • Gestión de frecuencias (ratio, multiplicidades)   │   │
│  │  • Conversión radianes ↔ encoder counts             │   │
│  │  • Offsets de lectura/escritura                     │   │
│  └──────────────────────────────────────────────────────┘   │
│                         │                                    │
│                         ▼                                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │           SteeringController                         │   │
│  │  ┌────────────────┐  ┌────────────────┐             │   │
│  │  │  MotorDriver   │  │ EncoderDriver  │             │   │
│  │  │  (Node ID: 1)  │  │ (Node ID: 127) │             │   │
│  │  │  • CiA 402     │  │  • Posición    │             │   │
│  │  │  • PDO/SDO     │  │    absoluta    │             │   │
│  │  │  • Control     │  │  • Filtrado    │             │   │
│  │  └────────────────┘  └────────────────┘             │   │
│  └──────────────────────────────────────────────────────┘   │
│                         │                                    │
│                         ▼                                    │
│  ┌──────────────────────────────────────────────────────┐   │
│  │         SocketCANInterface                           │   │
│  │  • Socket CAN RAW                                    │   │
│  │  • Epoll para lectura eficiente                     │   │
│  │  • Non-blocking I/O                                 │   │
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

## 📦 Componentes

### 1. **SystemSteeringHardware**
Hardware interface principal que implementa `hardware_interface::SystemInterface`.

**Características:**
- Gestión de frecuencias múltiples (controller manager vs hardware)
- Multiplicidades de lectura/escritura configurables
- Offsets independientes para read/write
- Conversión automática radianes ↔ encoder counts

### 2. **SteeringController**
Controlador de alto nivel que coordina motor y encoder.

**Funciones:**
- `init()`: Inicializa comunicación CAN y dispositivos CANopen
- `step()`: Ejecuta un ciclo de control (procesa CAN, actualiza estados)
- `setTargetSteeringPosition()`: Establece posición objetivo
- `getAbsoluteEncoderPosition()`: Lee posición del encoder absoluto
- `shutdown()`: Apaga el motor de forma segura

### 3. **MotorDriver** (CiA 402)
Driver para motor con protocolo CANopen CiA 402.

**Estados del motor:**
- `NOT_READY_TO_SWITCH_ON`
- `SWITCH_ON_DISABLED`
- `READY_TO_SWITCH_ON`
- `SWITCHED_ON`
- `OPERATION_ENABLED` ✓
- `FAULT`

**Funciones principales:**
- `initialize()`: Configura PDOs, TPDOs, NodeGuard
- `enableMotor()`: Habilita el motor (transición a OPERATION_ENABLED)
- `setTargetPosition()`: Envía posición objetivo
- `getActualPosition()`: Lee posición actual del encoder del motor

### 4. **EncoderDriver**
Driver para encoder absoluto externo.

**Funciones:**
- `initialize()`: Configura TPDOs y NodeGuard
- `getAbsolutePosition()`: Posición absoluta raw
- `getFilteredPosition()`: Posición filtrada
- `isValid()`: Estado de validez del encoder

### 5. **SocketCANInterface**
Interfaz de bajo nivel para comunicación CAN.

**Características:**
- Socket CAN en modo RAW
- Non-blocking I/O con epoll
- Lectura eficiente de múltiples frames
- Timeout configurable

### 6. **CANOpenDriver** (Clase base)
Clase base abstracta para dispositivos CANopen.

**Funcionalidades comunes:**
- Gestión de estados NMT
- Envío de SDO/PDO
- NodeGuard/Heartbeat
- Timeouts

## 🔧 Configuración

### Parámetros del Hardware Interface

```yaml
hardware:
  plugin: caddy_ai2_ros2_control_system_steering_driver/SystemSteeringHardware
  
  # Comunicación CAN
  interface_name: "can0"                    # Interfaz CAN (can0, vcan0, etc.)
  
  # Frecuencias
  controller_manager_frequency_hz: 500      # Frecuencia del controller manager
  hardware_sample_frequency_hz: 50          # Frecuencia del hardware CAN
  
  # Multiplicidades (cuántos ciclos esperar antes de leer/escribir)
  read_multiplicity: 1                      # Multiplicidad base de lectura
  write_multiplicity: 1                     # Multiplicidad base de escritura
  
  # Offsets (retraso en ciclos)
  read_offset: 0                            # Offset de lectura en ciclos
  write_offset: 0                           # Offset de escritura en ciclos
  
  # Parámetros CANopen
  motor_node_id: 1                          # Node ID del motor
  encoder_node_id: 127                      # Node ID del encoder
  counts_per_radian: 100000.0               # Factor de conversión
```

## 🔌 Configuración de Hardware Real

### Paso 1: Configurar adaptador USB-CAN

El paquete incluye un script interactivo para configurar automáticamente tu adaptador USB-CAN:

```bash
cd ~/ws_caddy_dev_ros2/src/caddy_ai2_ros2_control_system_steering_driver
chmod +x scripts/setup_can_steer_drv.sh
./scripts/setup_can_steer_drv.sh

### Cálculo de Frecuencias

El sistema calcula automáticamente:

```
frequency_ratio = controller_manager_frequency_hz / hardware_sample_frequency_hz
effective_read_multiplicity = read_multiplicity × frequency_ratio
effective_write_multiplicity = write_multiplicity × frequency_ratio

Frecuencia real de lectura = controller_manager_frequency_hz / effective_read_multiplicity
Frecuencia real de escritura = controller_manager_frequency_hz / effective_write_multiplicity
```

**Ejemplo:**
- Controller manager: 500 Hz
- Hardware: 50 Hz
- Read multiplicity: 1
- Write multiplicity: 1

```
frequency_ratio = 500 / 50 = 10
effective_read_multiplicity = 1 × 10 = 10
effective_write_multiplicity = 1 × 10 = 10

Frecuencia real lectura = 500 / 10 = 50 Hz ✓
Frecuencia real escritura = 500 / 10 = 50 Hz ✓
```

### Offsets de Lectura/Escritura

Los offsets permiten desfasar las operaciones de lectura y escritura:

```
Timeline (ciclos del controller manager @ 500 Hz):

Ciclo:  0   1   2   3   4   5   6   7   8   9   10  11  12
        │   │   │   │   │   │   │   │   │   │   │   │   │
Read:   ─   ─   R   ─   ─   ─   ─   ─   ─   ─   R   ─   ─   (offset=2, mult=10)
Write:  W   ─   ─   ─   ─   ─   ─   ─   ─   ─   W   ─   ─   (offset=0, mult=10)
```

## 🚀 Instalación

### Dependencias

```bash
# ROS2 Humble
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers

# CAN tools
sudo apt install can-utils

# Compilación
sudo apt install build-essential cmake
```

### Compilar el paquete

```bash
cd ~/ws_caddy_dev_ros2
colcon build --packages-select caddy_ai2_ros2_control_system_steering_driver
source install/setup.bash
```

## 🧪 Pruebas

### 1. Configurar CAN Virtual

```bash
# Cargar módulo vcan
sudo modprobe vcan

# Crear interfaz virtual
sudo ip link add dev vcan_steer_drv type vcan
sudo ip link set up vcan_steer_drv

# Verificar
ip link show vcan_steer_drv
```

O usar el script proporcionado:

```bash
sudo ./scripts/setup_vcan_steer_drv.sh
```

### 2. Monitorear CAN (terminal separada)

```bash
candump vcan_steer_drv
```

### 3. Lanzar el sistema

```bash
ros2 launch caddy_ai2_ros2_control_system_steering_driver virtual_system_steering.launch.py
```

### 4. Enviar comandos de prueba

```bash
# Publicar posición objetivo (radianes)
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5]"

# Ver estado actual
ros2 topic echo /joint_states
```

## 🔌 Hardware Real

### Configurar interfaz CAN física

```bash
# Configurar bitrate (ejemplo: 500 kbps)
sudo ip link set can0 type can bitrate 500000

# Activar interfaz
sudo ip link set up can0

# Verificar
ip -details link show can0
```

### Actualizar configuración

Edita `description/ros2_control/system_steering.ros2_control.urdf`:

```xml
<param name="interface_name">can0</param>  <!-- Cambiar de vcan_steer_drv a can0 -->
```

### Ajustar parámetros CANopen

Según tu hardware específico:

```xml
<param name="motor_node_id">1</param>           <!-- Node ID del motor -->
<param name="encoder_node_id">127</param>       <!-- Node ID del encoder -->
<param name="counts_per_radian">100000.0</param> <!-- Ajustar según resolución -->
```

**Cálculo de `counts_per_radian`:**

```
counts_per_radian = (encoder_resolution × gear_ratio) / (2 × π)

Ejemplo:
- Encoder: 4096 counts/rev
- Gear ratio: 154:1
- counts_per_radian = (4096 × 154) / (2 × π) ≈ 100,330
```

## 📊 Monitoreo y Diagnóstico

### Ver logs del hardware

```bash
ros2 run rqt_console rqt_console
```

### Inspeccionar estado del controller

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### Verificar frecuencias

Los logs muestran:
```
[SystemSteeringHardware] === Configuración del Hardware de Dirección ===
[SystemSteeringHardware] Interfaz CAN: can0
[SystemSteeringHardware] Frecuencia controller_manager: 500.00 Hz
[SystemSteeringHardware] Frecuencia del hardware: 50.00 Hz
[SystemSteeringHardware] Ratio de frecuencias: 10
[SystemSteeringHardware] Frecuencia real - Lectura: 50.00 Hz, Escritura: 50.00 Hz
```

### Herramientas CAN

```bash
# Ver mensajes CAN en tiempo real
candump can0

# Enviar mensaje CAN manual
cansend can0 181#0000000000000000

# Estadísticas de la interfaz
ip -s link show can0

# Ver errores CAN
cat /sys/class/net/can0/statistics/tx_errors
```

## ⚡ Optimización para Tiempo Real

### 1. Kernel RT-PREEMPT

```bash
# Instalar kernel RT
sudo apt install linux-image-rt-amd64

# Verificar
uname -a  # Debe mostrar "PREEMPT RT"
```

### 2. Configurar parámetros del kernel

Edita `/etc/default/grub`:

```bash
GRUB_CMDLINE_LINUX_DEFAULT="quiet splash \
    isolcpus=2,3 \
    nohz_full=2,3 \
    rcu_nocbs=2,3 \
    intel_pstate=disable \
    processor.max_cstate=1 \
    idle=poll"
```

Actualizar GRUB:

```bash
sudo update-grub
sudo reboot
```

### 3. Asignar CPUs aisladas al controller manager

```bash
# Lanzar con taskset
taskset -c 2 ros2 launch caddy_ai2_ros2_control_system_steering_driver virtual_system_steering.launch.py
```

### 4. Prioridad de proceso

```bash
# Ejecutar con prioridad RT
sudo chrt -f 80 ros2 launch ...
```

### 5. Medir latencia

```bash
# Instalar herramientas
sudo apt install rt-tests

# Medir latencia en CPU aislada
sudo cyclictest -p 80 -t1 -n -i 1000 -l 100000 -a 2
```

**Objetivo:**
- Min: < 10 µs
- Avg: < 20 µs
- Max: < 100 µs

## 📁 Estructura del Proyecto

```
caddy_ai2_ros2_control_system_steering_driver/
├── bringup/
│   ├── config/
│   │   └── system_steering.yaml              # Configuración de controladores
│   └── launch/
│       └── virtual_system_steering.launch.py # Launch file
├── description/
│   ├── ros2_control/
│   │   └── system_steering.ros2_control.urdf # Configuración hardware interface
│   └── urdf/
│       └── system_steering.urdf.xacro        # Descripción URDF
├── include/
│   └── caddy_ai2_ros2_control_system_steering_driver/
│       ├── system_steering_hardware.hpp      # Hardware interface
│       ├── steering_controller.hpp           # Controlador de dirección
│       ├── motor_driver.hpp                  # Driver motor CANopen
│       ├── encoder_driver.hpp                # Driver encoder CANopen
│       ├── canopen_driver.hpp                # Clase base CANopen
│       └── socket_can_interface.hpp          # Interfaz SocketCAN
├── src/
│   ├── system_steering_hardware.cpp
│   ├── steering_controller.cpp
│   ├── motor_driver.cpp
│   ├── encoder_driver.cpp
│   ├── canopen_driver.cpp
│   └── socket_can_interface.cpp
├── scripts/
│   └── setup_vcan_steer_drv.sh              # Script configuración CAN virtual
├── caddy_ai2_ros2_control_system_steering_driver.xml  # Plugin description
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 🐛 Troubleshooting

### Error: "No existe el archivo o el directorio can0"

```bash
# Verificar interfaces CAN disponibles
ip link show

# Si no existe, crear interfaz virtual
sudo ip link add dev can0 type vcan
sudo ip link set up can0
```

### Error: "Operation not permitted" al configurar CAN

```bash
# Ejecutar con sudo
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

### Motor no responde

1. Verificar Node ID correcto
2. Comprobar bitrate del CAN bus
3. Verificar cableado CAN (CANH, CANL, GND)
4. Revisar terminación del bus CAN (120Ω en ambos extremos)
5. Monitorear con `candump` para ver si hay tráfico

### Encoder no válido

1. Verificar alimentación del encoder
2. Comprobar Node ID
3. Revisar configuración de TPDOs
4. Verificar que el encoder esté en modo OPERATIONAL

### Latencia alta

1. Verificar frecuencias configuradas
2. Reducir `hardware_sample_frequency_hz` si es muy alta
3. Aplicar optimizaciones de tiempo real (ver sección anterior)
4. Verificar carga del sistema: `htop`

### Errores de compilación

```bash
# Limpiar build
cd ~/ws_caddy_dev_ros2
rm -rf build/ install/ log/

# Recompilar
colcon build --packages-select caddy_ai2_ros2_control_system_steering_driver --cmake-clean-cache
```

## 📚 Referencias

- [ROS2 Control Documentation](https://control.ros.org/)
- [CANopen CiA 402 Specification](https://www.can-cia.org/can-knowledge/canopen/cia402/)
- [SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)
- [RT-PREEMPT Howto](https://wiki.linuxfoundation.org/realtime/start)

## 📝 TODO

- [ ] Implementar control de velocidad
- [ ] Añadir límites de posición configurables
- [ ] Implementar safety stops
- [ ] Añadir diagnósticos extendidos
- [ ] Soporte para múltiples motores
- [ ] Calibración automática de `counts_per_radian`
- [ ] Interfaz de configuración dinámica (dynamic_reconfigure)
- [ ] Tests unitarios
- [ ] Documentación de la API

## 👥 Autores

- **Desarrollador Principal**: Rafael Carbonell Lázaro (racarla96)
- **Proyecto**: Caddy AI2 - Proyecto CERVAREC

## 📄 Licencia

Copyright (c) 2025, Rafael Carbonell Lázaro (racarla96)

Este proyecto se distribuye bajo la licencia **Creative Commons Attribution 4.0 International (CC BY 4.0)**.

### En resumen:

✅ **Puedes:**
- Usar, modificar y redistribuir la librería
- Utilizarla en proyectos comerciales o privados
- Crear trabajos derivados

⚠️ **Debes:**
- Mantener atribución al autor/proyecto (en documentación, créditos, "About" de la aplicación, etc.)
- Indicar si se realizaron cambios
- Proporcionar un enlace a la licencia

❌ **No puedes:**
- Imponer restricciones adicionales que impidan a otros ejercer los permisos que otorga la licencia

### Texto legal completo:
https://creativecommons.org/licenses/by/4.0/legalcode

### Atribución sugerida:


Este proyecto utiliza "caddy_ai2_ros2_control_system_steering_driver"
desarrollado por Rafael Carbonell Lázaro (racarla96)
Licencia: CC BY 4.0 (https://creativecommons.org/licenses/by/4.0/)
