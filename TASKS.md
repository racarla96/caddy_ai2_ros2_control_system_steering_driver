# Estado del desarrollo — caddy_ai2_ros2_control_system_steering_driver

Reescritura completa del paquete desde cero según la especificación
de arquitectura entregada al asistente. Sesión pausada; reanudar desde el paso 7.

---

## Progreso

| # | Archivo(s) | Estado |
|---|-----------|--------|
| 1 | `include/…/dzcante020l080_constants.hpp` | ✅ Completo |
| 2 | `include/…/canopen_driver.hpp` + `src/canopen_driver.cpp` | ✅ Completo |
| 3 | `include/…/motor_driver.hpp` + `src/motor_driver.cpp` | ✅ Completo |
| 4 | `include/…/encoder_driver.hpp` + `src/encoder_driver.cpp` | ✅ Completo |
| 5 | `include/…/steering_controller.hpp` + `src/steering_controller.cpp` | ✅ Completo |
| 6 | `test/test_standalone.cpp` + `README.md` | ✅ Completo |
| **7** | `include/…/system_steering_hardware.hpp` + `src/system_steering_hardware.cpp` | ⬜ Pendiente |
| **8** | `CMakeLists.txt` | ⬜ Pendiente |

---

## Paso 7 — system_steering_hardware (PENDIENTE)

### Parámetros URDF a leer en `on_init()`

```
interface_name                   (string)
motor_node_id                    (int)
encoder_node_id                  (int)
controller_manager_frequency_hz  (double)
hardware_sample_frequency_hz     (double)
read_multiplicity                (int, default 1)
write_multiplicity               (int, default 1)
read_offset                      (int, default 0)
write_offset                     (int, default 1)
```

Derivados calculados en `on_init()`:
```
frequency_ratio              = (int)(ctrl_hz / hw_hz)   ; mínimo 1
effective_read_multiplicity  = read_multiplicity  × frequency_ratio
effective_write_multiplicity = write_multiplicity × frequency_ratio
```

### Lifecycle

```
on_init       → leer params, calcular ratios, resize hw_states/hw_commands a NaN
on_configure  → new SteeringController(...) + sc->init()   ← SLEEPS AQUÍ
on_activate   → verificar sc->is_ready() con reintentos max 5 s; si no → FAILURE
                inicializar contadores con offsets:
                  read_counter_  = effective_read_multiplicity_  - read_offset_
                  write_counter_ = effective_write_multiplicity_ - write_offset_
on_deactivate → sc->shutdown()
on_error      → sc->shutdown(), reset contadores
```

### read()

```cpp
read_counter_++;
if (read_counter_ >= effective_read_multiplicity_) {
    read_counter_ = 0;
    sc->cycle_read(period.seconds());
    if (sc->has_fault()) return ERROR;
    hw_states_[0] = static_cast<double>(sc->get_position()) / counts_per_radian_;
}
return OK;
```

### write()

```cpp
write_counter_++;
if (write_counter_ >= effective_write_multiplicity_) {
    write_counter_ = 0;
    if (!isnan(hw_commands_[0])) {
        int32_t counts = static_cast<int32_t>(hw_commands_[0] * counts_per_radian_);
        sc->set_target_position(counts);
        sc->cycle_write();
    }
}
return OK;
```

### Interfaces

```
State interface:   joint_name / position  → hw_states_[0]
Command interface: joint_name / position  → hw_commands_[0]
```

### Miembros privados relevantes

```cpp
std::string interface_name_;
uint8_t     motor_node_id_;
uint8_t     encoder_node_id_;
double      controller_manager_frequency_hz_;
double      hardware_sample_frequency_hz_;
int         read_multiplicity_, write_multiplicity_;
int         read_offset_,       write_offset_;
int         frequency_ratio_;
int         effective_read_multiplicity_, effective_write_multiplicity_;
int         read_counter_,  write_counter_;
std::vector<double> hw_states_, hw_commands_;
std::unique_ptr<SteeringController> steering_controller_;
double      counts_per_radian_;   // ← obtener de param URDF o hardcodeado
```

### Nota sobre counts_per_radian_

El valor actual en el código viejo está hardcodeado como campo de clase.
Valorar si añadirlo como parámetro URDF o dejarlo como constante.
El URDF actual **no** incluye `motor_node_id` ni `encoder_node_id`: hay que añadirlos.

---

## Paso 8 — CMakeLists.txt (PENDIENTE)

Cambios respecto al CMakeLists actual:

1. **Añadir `dzcante020l080_constants.hpp`** al target (solo header, ya está en include/)

2. **Librería compartida** — mismos sources que ahora:
   ```cmake
   add_library(${PROJECT_NAME} SHARED
     src/canopen_driver.cpp
     src/motor_driver.cpp
     src/encoder_driver.cpp
     src/steering_controller.cpp
     src/system_steering_hardware.cpp
   )
   ```

3. **Añadir ejecutable `test_standalone`** — SIN dependencias de ROS:
   ```cmake
   add_executable(test_standalone
     test/test_standalone.cpp
     src/canopen_driver.cpp
     src/motor_driver.cpp
     src/encoder_driver.cpp
     src/steering_controller.cpp
   )
   target_include_directories(test_standalone PUBLIC
     $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
     ${caddy_ai2_ros2_common_INCLUDE_DIRS}
   )
   target_link_libraries(test_standalone
     caddy_ai2_ros2_common::caddy_ai2_ros2_common   # o la forma que use el paquete
   )
   install(TARGETS test_standalone DESTINATION lib/${PROJECT_NAME})
   ```
   **Importante**: `test_standalone` NO enlaza con `hardware_interface`, `rclcpp` ni `pluginlib`.

4. **Verificar** que `caddy_ai2_ros2_common` exporta correctamente su target para poder
   enlazar `test_standalone` sin ROS (quizás sea solo `-lpthread` o el .so de socket_can).

---

## Decisiones de diseño tomadas (resumen para retomar contexto)

| Decisión | Detalle |
|----------|---------|
| `dispatch_frame()` público en base | `process_frame()` es protected; el SC llama `motor_.dispatch_frame(frame)` |
| SYNC siempre primero en `cycle_read` | Sin SYNC el drive no emite TPDO1/TPDO21 |
| Watchdog incondicional en `update(dt)` | Se reinicia con `reset_watchdog()` al recibir heartbeat OPERATIONAL |
| `motor_.shutdown()` envía CW_QUICK_STOP inmediato | No usa `request_state()`: el socket se cierra justo después |
| Encoder usa COB-ID estándar 0x180 | Motor AMC usa propietario 0x4A0 para TPDO1 |
| NodeGuard usa `CAN_RTR_FLAG` | Frame RTR: `can_id = (0x700+nid) \| CAN_RTR_FLAG`, `dlc=1` |
| SDO command byte por tamaño | 0x2F=1B, 0x2B=2B, 0x23=4B (no siempre 0x23) |
| `configure()` solo en `on_configure` | Los sleeps no están en el bucle de control |
| `desired_state_` inicia en OPERATION_ENABLED | `tick_ds402` siempre intenta llegar ahí |

---

## Archivos que NO se tocan (intactos)

- `scripts/setup_can_steer_drv.sh`
- `scripts/setup_vcan_steer_drv.sh`
- `bringup/launch/system_steering.launch.py`
- `bringup/launch/virtual_system_steering.launch.py`
- `bringup/config/system_steering.yaml`
- `caddy_ai2_ros2_control_system_steering_driver.xml`
- `package.xml`
- `LICENSE`

**Sí necesita actualización** el URDF:
`description/ros2_control/system_steering.ros2_control.urdf`
→ añadir `<param name="motor_node_id">1</param>` y `<param name="encoder_node_id">127</param>`

---

## Para reanudar

Decir al asistente:
> "Retoma el desarrollo del steering driver desde el paso 7. Lee el archivo TASKS.md
> del paquete y los archivos actuales antes de continuar."
