# caddy_ai2_ros2_control_system_steering_driver

Driver de dirección para **ROS2 Control** sobre CANopen.  
Controla un motor **DZCANTE-020L080** de AMC (CiA 402, Profile Position) y soporta dos variantes de feedback de posición:

| Variante | Feedback | Fuente |
|---|---|---|
| **Potenciómetro** | Entrada analógica del DZCANTE | SDO polling `0x201A:subindex` |
| **Encoder EPC** | Encoder absoluto CANopen externo | TPDO1 estándar `0x180 + enc_node_id` |

El paquete es **auto-contenido**: `SocketCANInterface` está incluido, sin dependencia de `caddy_ai2_ros2_common`.  
Incluye tres ejecutables de test que funcionan sin el stack de ROS2.

---

## Arquitectura

```
ROS2 Controller Manager (500 Hz)
        │
        ▼
SystemSteeringHardware          ← hardware_interface::SystemInterface
  read()  → cycle_read(dt)      ← SDO poll + SYNC → drain CAN → update FSMs
  write() → cycle_write()       ← set target position (ASYNC RPDO21)
        │
        ▼
SteeringController              ← sin ROS, sin hilos propios
  ├── MotorDriver               ← CANopen CiA 402 (DZCANTE-020L080)
  │     • NMT watchdog + NodeGuard periódico
  │     • DS402 state machine (tick_ds402)
  │     • TPDO1  StatusWord    / TPDO21 ActualPosition
  │     • RPDO1  ControlWord   / RPDO21 TargetPosition
  │     • SDO 0x201A → entradas analógicas AI1/AI2/AI3  (variante POT)
  └── EncoderDriver (*)         ← encoder absoluto CAN externo (variante EPC)
        • NMT watchdog
        • TPDO1 posición absoluta (4 bytes LE)
        │
        ▼
SocketCANInterface              ← socket RAW + epoll, non-blocking
        │
        ▼
     CAN Bus (can_steer_drv / vcan0)
        │
   ┌────┴──────────────────┐
Motor (node 1)      Encoder (*) (node 127)
```

`(*) EncoderDriver solo presente en la variante EPC encoder.`

---

## COB-IDs y objetos SDO del DZCANTE-020L080

El drive AMC usa un mapping **propietario**, distinto al estándar CANopen:

| Objeto | Dir. | COB-ID / Índice | Contenido | Trans. |
|--------|------|-----------------|-----------|--------|
| RPDO1  | master→drive | `0x180 + node_id` | ControlWord (2 B) | ASYNC |
| RPDO21 | master→drive | `0x280 + node_id` | TargetPosition (int32 LE) | ASYNC |
| TPDO1  | drive→master | `0x4A0 + node_id` | StatusWord (2 B) | cada 10 SYNCs |
| TPDO21 | drive→master | `0x400 + node_id` | ActualPosition (int32 LE) | cada SYNC |
| NodeGuard RTR | master→drive | `(0x700+node_id)\|RTR` | — | periódico |
| **SDO AI req.**  | master→drive | `0x600 + node_id` | upload req. `0x201A:pin+1` | polling |
| **SDO AI resp.** | drive→master | `0x580 + node_id` | int16 LE, ±16384 → ±20 V | respuesta |

El encoder EPC externo usa TPDO1 **estándar** (`0x180 + enc_node_id`).

> **Nota:** el DZCANTE **no** emite TPDO4 (`0x480+nid`) con las entradas analógicas por defecto.
> La lectura de AI se hace por **SDO polling** al objeto propietario `0x201A` cada ciclo.
>
> Conversión: `mV = raw_int16 × 20000 / 16384`

---

## Ciclo de control

```
read()
  ├── [POT mode] motor.poll_analog_input(pin)   ← SDO upload req 0x201A:pin+1
  ├── send_sync()                                ← dispara TPDOs síncronos
  ├── receive_frames()                           ← drain no bloqueante (timeout=0)
  ├── motor.update(dt)                           ← watchdog NMT + tick_ds402
  └── [EPC mode] encoder.update(dt)              ← watchdog NMT

write()
  └── set_target_position(counts) + cycle_write()   ← RPDO21 si OPERATION_ENABLED
```

El SYNC va **siempre antes** del drain: sin él el drive no envía StatusWord ni ActualPosition.  
La respuesta SDO de `poll_analog_input()` llega en el siguiente `receive_frames()` (~20 ms).

---

## Máquinas de estado

**NMT** (watchdog 2 s — reiniciado por cada NodeGuard response):
```
UNKNOWN → PRE_OPERATIONAL → OPERATIONAL
                                  ↓ sin respuesta NodeGuard > 2 s
                                FAULT
```

**DS402** (`tick_ds402`, intervalo mínimo 50 ms):
```
SWITCH_ON_DISABLED → READY_TO_SWITCH_ON → OPERATION_DISABLED → OPERATION_ENABLED
       ↑                                                               ↓
   QUICK_STOP ←─────────────────────────────────────────────────────-─┘
       ↕
     FAULT  (auto-reset vía FAULT_RESET ControlWord)
```

---

## Calibración

### Motor (command: ángulo → counts)

```
actuator_zero        = (actuator_encoder_left  + actuator_encoder_right) / 2
actuator_counts/rad  = (actuator_encoder_left  - actuator_encoder_right) / (2 × range)
counts               = actuator_zero + angle_rad × actuator_counts/rad
```

Valores por defecto (rbcar): `left=99100`, `right=−97500` → `zero=800`, `scale≈171047 counts/rad`

### Feedback EPC (state: encoder counts → ángulo)

```
epc_rad/count  = −2 × range / (reading_encoder_right − reading_encoder_left)
angle_rad      = (epc_counts − reading_encoder_zero_position) × epc_rad/count
```

Signo negativo: count bajo → límite izquierdo → ángulo positivo.  
Valores por defecto (rbcar): `left=880`, `right=4520`, `zero=2650`, `range=0.5747 rad`

### Feedback potenciómetro (state: mV → ángulo)

```
pot_rad/mV  = −2 × range / (potentiometer_right_mv − potentiometer_left_mv)
angle_rad   = (pot_mv − potentiometer_zero_mv) × pot_rad/mV
```

> **Los valores `potentiometer_*_mv` son placeholders** y deben calibrarse físicamente.  
> Usa `test_passive` para leer los mV reales en cada límite y en posición recta.

---

## Parámetros URDF

```xml
<hardware>
  <plugin>caddy_ai2_ros2_control_system_steering_driver/SystemSteeringHardware</plugin>

  <!-- CAN -->
  <param name="can_interface_name">can_steer_drv</param>
  <param name="motor_node_id">1</param>

  <!-- Feedback: 0=potenciómetro, 1=encoder EPC -->
  <param name="feedback_mode">0</param>
  <param name="encoder_node_id">127</param>      <!-- solo si feedback_mode=1 -->
  <param name="analog_input_pin">2</param>       <!-- 0-based: 0=AI1,1=AI2,2=AI3 -->

  <!-- Calibración del actuador (counts del encoder del motor en cada límite físico) -->
  <param name="actuator_encoder_left">99100</param>
  <param name="actuator_encoder_right">-97500</param>

  <!-- Calibración encoder EPC (solo si feedback_mode=1) -->
  <param name="reading_encoder_left">880</param>
  <param name="reading_encoder_right">4520</param>
  <param name="reading_encoder_zero_position">2650</param>
  <param name="reading_encoder_resolution">4096</param>

  <!-- Calibración potenciómetro (solo si feedback_mode=0) — ajustar con test_passive -->
  <param name="potentiometer_left_mv">0</param>
  <param name="potentiometer_right_mv">5000</param>
  <param name="potentiometer_zero_mv">2500</param>

  <!-- Rango máximo de dirección (rad) -->
  <param name="steering_angle_range">0.5747</param>

  <!-- Temporización -->
  <param name="controller_manager_frequency_hz">500</param>
  <param name="hardware_sample_frequency_hz">500</param>
  <param name="read_multiplicity">1</param>    <!-- ciclos CM entre lecturas CAN -->
  <param name="write_multiplicity">10</param>  <!-- ciclos CM entre escrituras CAN -->
  <param name="read_offset">0</param>
  <param name="write_offset">1</param>
</hardware>
```

### Frecuencias efectivas

```
ratio          = ceil(controller_manager_hz / hardware_sample_hz)   (mín. 1)
read_rate_hz   = controller_manager_hz / (read_multiplicity  × ratio)
write_rate_hz  = controller_manager_hz / (write_multiplicity × ratio)
```

Con los valores por defecto (CM=500 Hz, HW=500 Hz, ratio=1):  
`read_rate = 500 Hz`, `write_rate = 50 Hz`

---

## Instalación

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers can-utils

cd ~/muppet_ws
colcon build --packages-select caddy_ai2_ros2_control_system_steering_driver
source install/setup.bash
```

---

## Configurar la interfaz CAN

### CAN virtual (sin hardware)

```bash
sudo ./install/caddy_ai2_ros2_control_system_steering_driver/lib/\
caddy_ai2_ros2_control_system_steering_driver/setup_vcan_steer_drv.sh
```

### Adaptador USB-CAN físico

```bash
./install/caddy_ai2_ros2_control_system_steering_driver/lib/\
caddy_ai2_ros2_control_system_steering_driver/setup_can_steer_drv.sh
# Interactivo: detecta el adaptador y crea la regla udev (nombre: can_steer_drv)
```

---

## Lanzar el sistema

```bash
# Hardware real
ros2 launch caddy_ai2_ros2_control_system_steering_driver system_steering.launch.py

# CAN virtual
ros2 launch caddy_ai2_ros2_control_system_steering_driver virtual_system_steering.launch.py
```

---

## Tests standalone (sin ROS)

Tres ejecutables independientes que no requieren el stack de ROS2.  
Ruta tras compilar: `./build/caddy_ai2_ros2_control_system_steering_driver/`

---

### `test_standalone` — monitor simple

Inicializa el drive, espera `OPERATION_ENABLED` y mantiene posición 0.  
Muestra estado y lectura de potenciómetro cada segundo. Smoke-test básico.

```bash
./test_standalone <can> [motor_id]

# Ejemplo:
./test_standalone can_steer_drv 1
```

Salida:
```
=== Steering Driver — Test Simple ===
  CAN       : can_steer_drv
  motor_id  : 1

[init] OK
[espera] Listo

t_s     motor_pos   pot_mV  NMT           DS402
----------------------------------------------------
1.0     312         2487    OPERATIONAL   OP_ENA
2.0     310         2489    OPERATIONAL   OP_ENA
```

---

### `test_calibration` — calibración del potenciómetro

Calcula automáticamente los tres parámetros `potentiometer_*_mv` del URDF.  
Usa regresión lineal sobre los pares `(motor_pos, AI_mV)` recogidos mientras mueves la rueda.

```bash
./test_calibration <can> [motor_id] [analog_pin]

# Ejemplo: potenciómetro en AI1 (pin 0)
./test_calibration can_steer_drv 1 0
```

**Procedimiento:**
1. Pon la rueda **recta** antes de ejecutar el programa — el primer valor leído se registra como referencia de 0 rad.
2. Mueve la rueda lentamente de un límite físico al otro (varios ciclos si es posible).
3. Pulsa Ctrl+C.

Salida mientras se ejecuta:
```
[zero] Referencia 0 rad capturada:  motor_pos=312  AI_mV=2487

motor_pos   AI_mV    |  min_pos     max_pos     min_mV    max_mV    N
------------------------------------------------------------------------------
79051       3372     |  -97500      99100       200       3800      6250
```

Salida al salir:
```
============================================================
  N muestras : 6250

  motor_pos  min=-97500  max=99100
  AI1_mV     min=200     max=3800

  Regresión lineal:
    AI_mV = 0.00962 × motor_pos + 2419.3
    Pearson r = 0.9991  (excelente)

  Parámetros URDF sugeridos
  --------------------------------------------------------
  potentiometer_left_mv   = 3372
  potentiometer_right_mv  = 1479
  potentiometer_zero_mv   = 2487  (medido)
============================================================
```

Copia los tres valores en [system_steering.ros2_control.urdf](description/ros2_control/system_steering.ros2_control.urdf) y recompila.

---

### `test_passive` — escucha sin mover el motor

Envía NMT START y SYNC pero **no ejecuta la secuencia DS402**.  
Imprime todos los frames recibidos y un resumen de valores cada segundo.  
**Imprescindible para calibrar los valores `potentiometer_*_mv`.**

```bash
./test_passive <can> [motor_id] [analog_pin]

# Ejemplo: leer AI3 (pin 2) del motor en nodo 1
./test_passive can_steer_drv 1 2
```

Salida:
```
  0x401  dlc=4  [28 01 00 00 -- -- -- --]  TPDO21-POS  pos=296
  0x581  dlc=8  [4b 1a 20 03 80 09 00 00]  SDO-RSP  cmd=0x4b
  0x4a1  dlc=2  [27 06 -- -- -- -- -- --]  TPDO1-SW  sw=0x627
  0x701  dlc=1  [05 -- -- -- -- -- -- --]  NMT  state=OPERATIONAL(0x5)
----------------------------------------------------------------------------
[Resumen t=1.0s]
  AI1=2487mV  AI2=155mV  AI3=12mV  --> pin2=12mV
  motor_pos=296
  status_word=0x627
```

Mueve la rueda hasta cada límite físico y anota el valor `-->` para actualizar  
`potentiometer_left_mv`, `potentiometer_right_mv` y `potentiometer_zero_mv` en el URDF.

---

### `test_sinusoidal` — seguimiento senoidal activo

Hace seguir al motor una referencia sinusoidal entre dos límites en counts.  
Soporta feedback por potenciómetro (`fb_mode=0`) o encoder EPC (`fb_mode=1`).

```bash
./test_sinusoidal <can> <motor_id> <fb_mode> <fb_param> <min> <max> [freq_hz] [cycles]

#   fb_mode  0 = potenciómetro
#   fb_mode  1 = encoder EPC
#   fb_param si fb_mode=0: pin analógico 0-based (0=AI1, 1=AI2, 2=AI3)
#   fb_param si fb_mode=1: node_id del encoder

# Potenciómetro en AI3, rango ±90000 counts, 0.05 Hz × 3 ciclos:
./test_sinusoidal can_steer_drv 1 0 2 -90000 90000 0.05 3

# Encoder EPC en nodo 127:
./test_sinusoidal can_steer_drv 1 1 127 -90000 90000 0.05 3
```

Salida:
```
t_s    ref       motor_pos   error     pot_mV      NMT           DS402
---------------------------------------------------------------------------
0.20   0         312         -312      2487        OPERATIONAL   OP_ENA
0.40   2827      3105        -278      2651        OPERATIONAL   OP_ENA
```

---

## Monitoreo CAN

```bash
# Todo el tráfico
candump can_steer_drv

# Solo TPDOs del motor (node_id=1)
candump can_steer_drv,4A1:7FF,401:7FF

# Respuestas SDO con valores analógicos (0x201A)
candump can_steer_drv,581:7FF

# Estadísticas de la interfaz
ip -s link show can_steer_drv
```

---

## Enviar comandos de posición

El controlador es un `JointGroupPositionController` que escucha en:

```
/steering/system_steering_controller/commands   (std_msgs/msg/Float64MultiArray)
```

El valor es en **radianes**. Límites del joint: `[-0.4, 0.4]` rad.

```bash
# Posición recta
ros2 topic pub --once /steering/system_steering_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.0]"

# Girar a la izquierda (~20°)
ros2 topic pub --once /steering/system_steering_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.35]"

# Girar a la derecha (~20°)
ros2 topic pub --once /steering/system_steering_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [-0.35]"

# Envío continuo a 10 Hz (mantiene la posición activa)
ros2 topic pub -r 10 /steering/system_steering_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [0.0]"
```

## Diagnóstico con ros2_control

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /steering/joint_states
```

---

## Estructura del paquete

```
caddy_ai2_ros2_control_system_steering_driver/
├── include/…/
│   ├── dzcante020l080_constants.hpp   ← COB-IDs, CW, SDO (0x201A), timeouts
│   ├── socket_can_interface.hpp       ← SocketCAN RAW + epoll (paquete auto-contenido)
│   ├── canopen_driver.hpp             ← base: NMT, NodeGuard, SDO/PDO primitives
│   ├── motor_driver.hpp               ← CiA 402, DS402 FSM, SDO analog (0x201A)
│   ├── encoder_driver.hpp             ← encoder absoluto CAN externo
│   ├── steering_controller.hpp        ← fachada: FeedbackSource, cycle_read/write
│   └── system_steering_hardware.hpp   ← hardware_interface::SystemInterface
├── src/
│   ├── socket_can_interface.cpp
│   ├── canopen_driver.cpp
│   ├── motor_driver.cpp
│   ├── encoder_driver.cpp
│   ├── steering_controller.cpp
│   └── system_steering_hardware.cpp
├── test/
│   ├── test_standalone.cpp            ← monitor simple, posición 0 (sin ROS)
│   ├── test_passive.cpp               ← escucha sin DS402, calibración POT
│   └── test_sinusoidal.cpp            ← seguimiento senoidal activo
├── bringup/
│   ├── config/system_steering.yaml
│   └── launch/
│       ├── system_steering.launch.py
│       └── virtual_system_steering.launch.py
├── description/
│   ├── ros2_control/system_steering.ros2_control.urdf
│   └── urdf/system_steering.urdf.xacro
├── scripts/
│   ├── setup_can_steer_drv.sh
│   └── setup_vcan_steer_drv.sh
└── manual_up_can_interface.md
```

---

## Troubleshooting

### Motor no responde tras `init()`

- Verificar con `candump` que llegan frames NMT (`000#`) y SDO (`601#` para nodo 1)
- Comprobar que el drive responde al NodeGuard RTR: debe aparecer `701#05` o `701#85`
- Verificar bitrate del bus CAN (el DZCANTE suele ir a 1 Mbps)
- Revisar terminación del bus (120 Ω en cada extremo)

### NMT pasa a FAULT tras 2 s

- El NodeGuard RTR no recibe respuesta: `candump` debe mostrar `701#R` (RTR enviada) y `701#05`/`701#85` (respuesta del drive)
- Comprobar que el guard time está configurado en el drive (`0x100C > 0`)

### `pot_mV` siempre a 0 o no varía

- Verificar que el drive responde al SDO `0x201A`: `candump can_steer_drv,581:7FF`
- Comprobar el subindex: el driver envía `pin+1` (AI3 = subindex 3, `0x201A:3`)
- Usar `test_passive` para ver los frames raw y confirmar qué AI tiene el potenciómetro conectado

### Timeout en `on_activate` (OPERATION_ENABLED no alcanzado)

- El drive puede tener un FAULT previo: ejecutar `test_standalone` para ver el DS402 en detalle
- Comprobar que la tensión del bus DC del DZCANTE está dentro de rango
- En variante EPC: verificar que el encoder responde al SYNC (`candump` muestra `1FF#`)

### Encoder no válido (`is_valid() == false`)

- Verificar que el SYNC llega al encoder: `candump` debe mostrar `080#`
- Comprobar que el encoder responde con TPDO1 en `0x180 + enc_node_id`
- Default `enc_node_id=127` → frames en `0x1FF`

### Limpiar y recompilar

```bash
cd ~/muppet_ws
rm -rf build/caddy_ai2_ros2_control_system_steering_driver \
        install/caddy_ai2_ros2_control_system_steering_driver
colcon build --packages-select caddy_ai2_ros2_control_system_steering_driver
```

---

## Optimización tiempo real

El sistema usa kernel RT (`uname -a` debe mostrar `PREEMPT_RT`). Para el controller manager:

```bash
taskset -c 2 sudo chrt -f 80 ros2 launch \
  caddy_ai2_ros2_control_system_steering_driver system_steering.launch.py
```

Latencia objetivo con `cyclictest`: avg < 20 µs, max < 100 µs.

---

## Licencia

Copyright © 2025 Rafael Carbonell Lázaro (racarla96) — CC BY 4.0  
https://creativecommons.org/licenses/by/4.0/
