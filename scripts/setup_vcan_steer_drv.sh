#!/bin/bash

# setup_vcan_steer_drv.sh - Configurar interfaz CAN virtual

echo "Configurando interfaz CAN virtual..."

# Cargar módulo vcan si no está cargado
if ! lsmod | grep -q vcan_steer_drv; then
    echo "Cargando módulo vcan_steer_drv..."
    sudo modprobe vcan_steer_drv
fi

# Eliminar vcan_steer_drv si ya existe
if ip link show vcan_steer_drv &> /dev/null; then
    echo "Eliminando vcan_steer_drv existente..."
    sudo ip link delete vcan_steer_drv
fi

# Crear y activar vcan_steer_drv
echo "Creando vcan_steer_drv..."
sudo ip link add dev vcan_steer_drv type vcan
sudo ip link set up vcan_steer_drv

echo "✓ Interfaz vcan_steer_drv configurada correctamente"
ip link show vcan_steer_drv