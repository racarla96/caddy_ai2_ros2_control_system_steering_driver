# Tests standalone (sin ROS)

Cinco ejecutables independientes que no requieren el stack de ROS2.  
Los ejemplos de uso usan la ruta completa desde la raíz del workspace
(`colcon build` sin `--merge-install`):

```
./build/caddy_ai2_ros2_control_system_steering_driver/<test_name>
```

---

## `test_standalone` — monitor simple

Inicializa el drive, espera `OPERATION_ENABLED` y mantiene posición 0.  
Muestra estado y lectura de potenciómetro cada segundo. Smoke-test básico.

```bash
./build/caddy_ai2_ros2_control_system_steering_driver/test_standalone <can> [motor_id]

# Ejemplo:
./build/caddy_ai2_ros2_control_system_steering_driver/test_standalone can_steer_drv 1
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

## `test_passive` — escucha sin mover el motor

Envía NMT START y SYNC pero **no ejecuta la secuencia DS402**.  
Imprime todos los frames recibidos y un resumen de valores cada segundo.  
Útil para calibrar `potentiometer_*_mv` y verificar el encoder EPC.

```bash
./build/caddy_ai2_ros2_control_system_steering_driver/test_passive <can> [motor_id] [analog_pin] [encoder_id]

# encoder_id = 0  → sin encoder EPC (default)
# encoder_id = 127 → muestra también el TPDO1 del encoder EPC en nodo 127

# Leer AI3 (pin 2) del motor en nodo 1, sin encoder:
./build/caddy_ai2_ros2_control_system_steering_driver/test_passive can_steer_drv 1 2

# Leer AI3 + encoder EPC en nodo 127:
./build/caddy_ai2_ros2_control_system_steering_driver/test_passive can_steer_drv 1 2 127
```

Salida:
```
  0x401  dlc=4  [28 01 00 00 -- -- -- --]  TPDO21-POS  pos=296
  0x581  dlc=8  [4b 1a 20 03 80 09 00 00]  SDO-RSP  cmd=0x4b
  0x4a1  dlc=2  [27 06 -- -- -- -- -- --]  TPDO1-SW  sw=0x627
  0x701  dlc=1  [05 -- -- -- -- -- -- --]  NMT  state=OPERATIONAL(0x5)
  0x1ff  dlc=4  [a2 0a 00 00 -- -- -- --]  ENC-TPDO1  enc_pos=2722
----------------------------------------------------------------------------
[Resumen t=1.0s]
  AI1=2487mV  AI2=155mV  AI3=12mV  --> pin2=12mV
  motor_pos=296
  enc_pos=2722  (node 127)
  status_word=0x627
```

Mueve la rueda hasta cada límite físico y anota el valor `-->` (para potenciómetro)
o `enc_pos` (para encoder EPC) para actualizar los parámetros en el URDF.

---

## `test_calibration` — calibración del potenciómetro

Calcula automáticamente los tres parámetros `potentiometer_*_mv` del URDF.  
Usa regresión lineal sobre los pares `(motor_pos, AI_mV)` recogidos mientras
mueves la rueda. **No ejecuta DS402** — pasivo.

```bash
./build/caddy_ai2_ros2_control_system_steering_driver/test_calibration <can> [motor_id] [analog_pin]

# Ejemplo: potenciómetro en AI1 (pin 0)
./build/caddy_ai2_ros2_control_system_steering_driver/test_calibration can_steer_drv 1 0
```

**Procedimiento:**
1. Pon la rueda **recta** antes de ejecutar — el primer valor leído se registra como referencia de 0 rad.
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

Copia los tres valores en
[description/ros2_control/system_steering.ros2_control.urdf](../description/ros2_control/system_steering.ros2_control.urdf)
y recompila.

---

## `test_calibration_epc` — calibración del encoder EPC

Configura el TPDO1 del encoder vía SDO, luego lee en paralelo `motor_pos`
(TPDO21) y `enc_pos` (TPDO1 `0x180 + encoder_id`). Al salir imprime los cinco
parámetros URDF necesarios para la variante EPC. **No ejecuta DS402** — pasivo.

```bash
./build/caddy_ai2_ros2_control_system_steering_driver/test_calibration_epc <can> [motor_id] [encoder_id]

# Ejemplo: motor en nodo 1, encoder EPC en nodo 127
./build/caddy_ai2_ros2_control_system_steering_driver/test_calibration_epc can_steer_drv 1 127
```

**Procedimiento:**
1. Pon la rueda **recta** antes de ejecutar — el primer par válido se registra como zero.
2. Mueve la rueda lentamente entre los dos límites físicos.
3. Ctrl+C para ver los parámetros URDF sugeridos.

Salida mientras se ejecuta:
```
[cfg] Configurando encoder TPDO1...
[cfg] OK

[zero] Referencia 0 rad capturada:  motor_pos=312  enc_pos=2650

motor_pos   enc_pos  |  min_motor       max_motor     min_enc     max_enc     N
------------------------------------------------------------------------------------
-85000      3900     |  -97500          97750         880         4520        6250
```

Salida al salir:
```
============================================================
  N muestras : 6250

  motor_pos  min=-97500  max=97750
  enc_pos    min=880     max=4520

  Parámetros URDF sugeridos
  --------------------------------------------------------
  actuator_encoder_left         = 97750
  actuator_encoder_right        = -97500
  reading_encoder_left          = 880
  reading_encoder_right         = 4520
  reading_encoder_zero_position = 2650  (medido, motor_pos=312)
============================================================
```

Copia los valores en
[description/ros2_control/system_steering.ros2_control.urdf](../description/ros2_control/system_steering.ros2_control.urdf)
y recompila.

---

## `test_sinusoidal` — seguimiento senoidal activo

Hace seguir al motor una referencia sinusoidal entre dos límites en counts.  
Soporta feedback por potenciómetro (`fb_mode=0`) o encoder EPC (`fb_mode=1`).

```bash
./build/caddy_ai2_ros2_control_system_steering_driver/test_sinusoidal <can> <motor_id> <fb_mode> <fb_param> <min> <max> [freq_hz] [cycles]

#   fb_mode  0 = potenciómetro
#   fb_mode  1 = encoder EPC
#   fb_param si fb_mode=0: pin analógico 0-based (0=AI1, 1=AI2, 2=AI3)
#   fb_param si fb_mode=1: node_id del encoder

# Potenciómetro en AI3, rango ±90000 counts, 0.05 Hz × 3 ciclos:
./build/caddy_ai2_ros2_control_system_steering_driver/test_sinusoidal can_steer_drv 1 0 2 -90000 90000 0.05 3

# Encoder EPC en nodo 127:
./build/caddy_ai2_ros2_control_system_steering_driver/test_sinusoidal can_steer_drv 1 1 127 -90000 90000 0.05 3
```

Salida:
```
t_s    ref       motor_pos   error     enc_pos     NMT           DS402
---------------------------------------------------------------------------
0.20   0         312         -312      2650        OPERATIONAL   OP_ENA
0.40   2827      3105        -278      2680        OPERATIONAL   OP_ENA
```
