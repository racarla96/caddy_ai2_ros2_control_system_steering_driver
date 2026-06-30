# Instalación y configuración CAN

## Dependencias

```bash
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers can-utils
```

## Compilar el paquete

```bash
cd ~/ws_ros2_caddy
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

### Adaptador USB-CAN físico — IXXAT

Instalar previamente los drivers del sistema (ver [drivers/](drivers/)):

```bash
./install/caddy_ai2_ros2_control_system_steering_driver/lib/\
caddy_ai2_ros2_control_system_steering_driver/setup_ixxat_usb_can_steer_drv.sh
```

El script es interactivo: detecta el adaptador IXXAT y crea una regla udev que
asigna el nombre persistente `can_steer_drv` a la interfaz.

---

## Levantar la interfaz CAN manualmente

Si necesitas subir la interfaz sin usar los scripts (por ejemplo para pruebas
rápidas con un adaptador ya identificado):

```bash
# Adaptador real (ajustar bitrate si el drive no va a 1 Mbps)
sudo ip link set can_steer_drv type can bitrate 1000000
sudo ip link set can_steer_drv up

# can0 genérico
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

---

## Contenido de este directorio

```
install/
├── README.md                        ← este archivo
├── scripts/
│   ├── setup_ixxat_usb_can_steer_drv.sh   ← configura adaptador IXXAT + udev
│   └── setup_vcan_steer_drv.sh             ← crea interfaz CAN virtual
└── drivers/
    └── ixxat--socketcan-linux.gz           ← driver SocketCAN para IXXAT USB-CAN
```
