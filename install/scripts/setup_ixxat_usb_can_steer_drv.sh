#!/bin/bash

################################################################################
# Script de configuración de adaptador USB-CAN para sistema de dirección
# Proyecto: Caddy AI2 ROS2 Control System Steering Driver
# Autor: Rafael Carbonell Lázaro (racarla96)
# Licencia: CC BY 4.0
################################################################################

set -e  # Salir si hay error

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuración
INTERFACE_NAME="can_steer_drv"
BITRATE="1000000"  # 1 Mbps (ajustar según tu hardware)
UDEV_RULES_FILE="/etc/udev/rules.d/81-can-usb-steering.rules"

################################################################################
# Funciones auxiliares
################################################################################

print_header() {
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  Configuración de adaptador USB-CAN para dirección${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════════${NC}"
}

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_step() {
    echo -e "\n${BLUE}▶${NC} $1"
}

################################################################################
# Detección automática de adaptadores USB-CAN
################################################################################

detect_usb_can_adapters() {
    print_step "Detectando adaptadores USB-CAN conectados..."
    
    # Buscar dispositivos USB relacionados con CAN
    echo -e "\n${YELLOW}Dispositivos USB detectados:${NC}"
    lsusb | grep -i "can\|usb.*serial\|ch341\|ftdi" || echo "  (ninguno encontrado con palabras clave comunes)"
    
    echo -e "\n${YELLOW}Mensajes del kernel (dmesg):${NC}"
    echo "───────────────────────────────────────────────────────────────"
    dmesg | grep -i "usb\|can\|serial" | tail -20
    echo "───────────────────────────────────────────────────────────────"
    
    echo -e "\n${YELLOW}Interfaces CAN actuales:${NC}"
    ip link show | grep -E "can[0-9]|slcan[0-9]" || echo "  (ninguna interfaz CAN detectada)"
}

################################################################################
# Obtener información del adaptador
################################################################################

get_adapter_info() {
    print_step "Identificación del adaptador USB-CAN"
    
    echo -e "\n${YELLOW}Métodos de identificación disponibles:${NC}"
    echo "  1. Número de serie (SerialNumber) - Recomendado"
    echo "  2. ID del producto (idVendor:idProduct)"
    echo "  3. Ruta del bus USB (devpath)"
    
    echo -e "\n${YELLOW}Información detallada de dispositivos USB:${NC}"
    echo "───────────────────────────────────────────────────────────────"
    
    # Listar todos los dispositivos USB con información detallada
    for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        
        # Obtener atributos
        vendor=$(udevadm info -q property -p $syspath | grep "ID_VENDOR_ID=" | cut -d'=' -f2)
        product=$(udevadm info -q property -p $syspath | grep "ID_MODEL_ID=" | cut -d'=' -f2)
        serial=$(udevadm info -q property -p $syspath | grep "ID_SERIAL_SHORT=" | cut -d'=' -f2)
        model=$(udevadm info -q property -p $syspath | grep "ID_MODEL=" | cut -d'=' -f2)
        
        if [[ -n "$vendor" ]] && [[ -n "$product" ]]; then
            echo -e "${GREEN}Dispositivo:${NC} $devname"
            echo "  Vendor:Product = $vendor:$product"
            [[ -n "$model" ]] && echo "  Modelo: $model"
            [[ -n "$serial" ]] && echo "  Serial: $serial"
            echo ""
        fi
    done
    echo "───────────────────────────────────────────────────────────────"
}

################################################################################
# Configuración interactiva
################################################################################

configure_udev_rule() {
    print_step "Configuración de regla udev"
    
    echo -e "\n${YELLOW}Selecciona el método de identificación:${NC}"
    echo "  1) Número de serie (SerialNumber) - Más específico"
    echo "  2) ID del producto (idVendor:idProduct) - Todos los adaptadores del mismo modelo"
    read -p "Opción [1]: " METHOD
    METHOD=${METHOD:-1}
    
    case $METHOD in
        1)
            echo -e "\n${YELLOW}Introduce el número de serie del adaptador:${NC}"
            echo "  (Ejemplo: HW033197, FT1234AB, etc.)"
            read -p "Serial: " HWSERIAL
            
            if [[ -z "$HWSERIAL" ]]; then
                print_error "Número de serie vacío. Abortando."
                exit 1
            fi
            
            UDEV_RULE="SUBSYSTEM==\"net\", ACTION==\"add\", ATTRS{serial}==\"$HWSERIAL\", NAME=\"$INTERFACE_NAME\", RUN+=\"/bin/sh -c 'sleep 1 && /sbin/ip link set $INTERFACE_NAME type can bitrate $BITRATE && /sbin/ip link set up $INTERFACE_NAME'\""
            ;;
            
        2)
            echo -e "\n${YELLOW}Introduce el Vendor ID (ejemplo: 1a86):${NC}"
            read -p "Vendor ID: " VENDOR_ID
            echo -e "\n${YELLOW}Introduce el Product ID (ejemplo: 7523):${NC}"
            read -p "Product ID: " PRODUCT_ID
            
            if [[ -z "$VENDOR_ID" ]] || [[ -z "$PRODUCT_ID" ]]; then
                print_error "IDs vacíos. Abortando."
                exit 1
            fi
            
            UDEV_RULE="SUBSYSTEM==\"net\", ACTION==\"add\", ATTRS{idVendor}==\"$VENDOR_ID\", ATTRS{idProduct}==\"$PRODUCT_ID\", NAME=\"$INTERFACE_NAME\", RUN+=\"/bin/sh -c 'sleep 1 && /sbin/ip link set $INTERFACE_NAME type can bitrate $BITRATE && /sbin/ip link set up $INTERFACE_NAME'\""
            ;;
            
        *)
            print_error "Opción inválida. Abortando."
            exit 1
            ;;
    esac
    
    # Configurar bitrate
    echo -e "\n${YELLOW}Bitrate del bus CAN:${NC}"
    echo "  Valores comunes: 125000, 250000, 500000, 1000000"
    read -p "Bitrate [$BITRATE]: " INPUT_BITRATE
    BITRATE=${INPUT_BITRATE:-$BITRATE}
    
    # Actualizar regla con el bitrate seleccionado
    UDEV_RULE=$(echo "$UDEV_RULE" | sed "s/bitrate [0-9]*/bitrate $BITRATE/")
}

################################################################################
# Instalación de la regla
################################################################################

install_udev_rule() {
    print_step "Instalación de regla udev"
    
    echo -e "\n${YELLOW}Regla a instalar:${NC}"
    echo "───────────────────────────────────────────────────────────────"
    echo "$UDEV_RULE"
    echo "───────────────────────────────────────────────────────────────"
    echo -e "\n${YELLOW}Archivo:${NC} $UDEV_RULES_FILE"
    echo -e "${YELLOW}Interfaz:${NC} $INTERFACE_NAME"
    echo -e "${YELLOW}Bitrate:${NC} $BITRATE bps"
    
    read -p $'\n¿Deseas continuar con la instalación? (s/n): ' CONFIRM
    if [[ "$CONFIRM" != "s" ]]; then
        print_warning "Instalación cancelada."
        exit 0
    fi
    
    # Backup de regla existente
    if [[ -f "$UDEV_RULES_FILE" ]]; then
        print_info "Creando backup de regla existente..."
        sudo cp "$UDEV_RULES_FILE" "${UDEV_RULES_FILE}.backup.$(date +%Y%m%d_%H%M%S)"
    fi
    
    # Escribir regla
    print_info "Escribiendo regla udev..."
    echo "$UDEV_RULE" | sudo tee "$UDEV_RULES_FILE" > /dev/null
    
    # Recargar udev
    print_info "Recargando reglas udev..."
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    
    print_info "✓ Regla instalada correctamente"
}

################################################################################
# Verificación
################################################################################

verify_installation() {
    print_step "Verificación de la instalación"
    
    echo -e "\n${YELLOW}Para aplicar los cambios:${NC}"
    echo "  1. Desconecta el adaptador USB-CAN"
    echo "  2. Espera 2 segundos"
    echo "  3. Vuelve a conectarlo"
    
    read -p $'\n¿Deseas verificar ahora? (desconecta y reconecta el adaptador primero) (s/n): ' VERIFY
    
    if [[ "$VERIFY" == "s" ]]; then
        echo -e "\n${YELLOW}Esperando 5 segundos...${NC}"
        sleep 5
        
        echo -e "\n${YELLOW}Interfaces de red detectadas:${NC}"
        ip link show
        
        if ip link show "$INTERFACE_NAME" &> /dev/null; then
            print_info "✓ Interfaz $INTERFACE_NAME detectada correctamente"
            
            echo -e "\n${YELLOW}Estado de la interfaz:${NC}"
            ip -details link show "$INTERFACE_NAME"
            
            echo -e "\n${YELLOW}Estadísticas:${NC}"
            ip -s link show "$INTERFACE_NAME"
        else
            print_error "✗ Interfaz $INTERFACE_NAME NO detectada"
            echo -e "\n${YELLOW}Posibles causas:${NC}"
            echo "  - El adaptador no está conectado"
            echo "  - El número de serie o IDs son incorrectos"
            echo "  - El driver del adaptador no está cargado"
            echo "  - Revisa los logs: dmesg | tail -20"
        fi
    fi
}

################################################################################
# Información adicional
################################################################################

print_usage_info() {
    print_step "Información de uso"
    
    echo -e "\n${YELLOW}Comandos útiles:${NC}"
    echo "  # Ver estado de la interfaz"
    echo "  ip link show $INTERFACE_NAME"
    echo ""
    echo "  # Monitorear tráfico CAN"
    echo "  candump $INTERFACE_NAME"
    echo ""
    echo "  # Enviar mensaje CAN de prueba"
    echo "  cansend $INTERFACE_NAME 123#DEADBEEF"
    echo ""
    echo "  # Ver estadísticas"
    echo "  ip -s link show $INTERFACE_NAME"
    echo ""
    echo "  # Reconfigurar manualmente (si es necesario)"
    echo "  sudo ip link set $INTERFACE_NAME down"
    echo "  sudo ip link set $INTERFACE_NAME type can bitrate $BITRATE"
    echo "  sudo ip link set $INTERFACE_NAME up"
    echo ""
    echo "  # Ver logs del kernel"
    echo "  dmesg | grep -i can"
    echo ""
    echo "  # Verificar reglas udev"
    echo "  cat $UDEV_RULES_FILE"
    
    echo -e "\n${YELLOW}Integración con ROS2 Control:${NC}"
    echo "  Edita: description/ros2_control/system_steering.ros2_control.urdf"
    echo "  Cambia: <param name=\"interface_name\">$INTERFACE_NAME</param>"
}

################################################################################
# Main
################################################################################

main() {
    print_header
    
    # Verificar permisos
    if [[ $EUID -eq 0 ]]; then
        print_warning "No ejecutes este script como root. Se solicitará sudo cuando sea necesario."
        exit 1
    fi
    
    # Verificar dependencias
    if ! command -v udevadm &> /dev/null; then
        print_error "udevadm no encontrado. Instala: sudo apt install udev"
        exit 1
    fi
    
    # Pasos del script
    detect_usb_can_adapters
    get_adapter_info
    configure_udev_rule
    install_udev_rule
    verify_installation
    print_usage_info
    
    echo -e "\n${GREEN}═══════════════════════════════════════════════════════════════${NC}"
    echo -e "${GREEN}  ✓ Configuración completada${NC}"
    echo -e "${GREEN}═══════════════════════════════════════════════════════════════${NC}\n"
}

# Ejecutar
main