#include "caddy_ai2_ros2_control_system_steering_driver/encoder_driver.hpp"
#include "caddy_ai2_ros2_common/socket_can_interface.hpp"
#include <iostream>
#include <thread>

EncoderDriver::EncoderDriver(uint8_t node_id, const std::string& name)
    : CANOpenDriver(node_id, name)
    , absolute_position_(0)
    , filtered_position_(0)
    , encoder_ok_(false) {
}

// encoder_driver.cpp
// ...
bool EncoderDriver::initialize(SocketCANInterface* can_interface) {
    can_interface_ = can_interface;

    std::cout << "[" << name_ << "] Iniciando configuración encoder..." << std::endl;

    // Reset de comunicación
    if (!sendNMT(0x82)) {  // RESET_COMMUNICATION
        std::cerr << "[" << name_ << "] Error enviando RESET_COMMUNICATION" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Pre-Operational
    if (!sendNMT(0x80)) {
        std::cerr << "[" << name_ << "] Error enviando PRE_OPERATIONAL" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    nmt_state_ = NMTState::PRE_OPERATIONAL;

    // Configurar PDOs y nodeguard
    if (!configureTPDOs()) { // Llama al nuevo método configureTPDOs
        std::cerr << "[" << name_ << "] Error configurando TPDOs encoder" << std::endl;
        return false;
    }
    if (!configureNodeGuard()) {
        std::cerr << "[" << name_ << "] Error configurando NodeGuard encoder" << std::endl;
        return false;
    }

    std::cout << "[" << name_ << "] Configuración completada" << std::endl;
    return true;
}
// ...

// encoder_driver.cpp
// ...
bool EncoderDriver::configureTPDOs() {
    // Deshabilitar TPDO1 para configuración (bit 31 a 1)
    uint32_t tpdo1_cob_id_disable = 0x80000000 | (0x180 + node_id_);
    if (!sendSDO(0x1800, 0x01, (uint8_t*)&tpdo1_cob_id_disable, 4, true)) {
        std::cerr << "[" << name_ << "] Error deshabilitando TPDO1 para config" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Mapear TPDO1 (ejemplo: 0x6000:01 para posición absoluta)
    // Número de objetos mapeados (0x1600:00)
    uint8_t num_mapped_objects = 0;
    if (!sendSDO(0x1600, 0x00, &num_mapped_objects, 1, true)) {
        std::cerr << "[" << name_ << "] Error reseteando mapeo TPDO1" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Mapear 0x6000:01 (posición absoluta, 32 bits)
    uint32_t mapped_object_1 = 0x60000120; // Index: 0x6000, Subindex: 0x01, Size: 32 bits
    if (!sendSDO(0x1600, 0x01, (uint8_t*)&mapped_object_1, 4, true)) {
        std::cerr << "[" << name_ << "] Error mapeando objeto 1 TPDO1" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    num_mapped_objects = 1; // Un objeto mapeado
    if (!sendSDO(0x1600, 0x00, &num_mapped_objects, 1, true)) {
        std::cerr << "[" << name_ << "] Error configurando num objetos mapeados TPDO1" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Tipo de transmisión (0x1800:02) - 1 = síncrono (por SYNC)
    uint8_t transmission_type = 0x01;
    if (!sendSDO(0x1800, 0x02, &transmission_type, 1, true)) {
        std::cerr << "[" << name_ << "] Error configurando tipo transmisión TPDO1" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Habilitar TPDO1 (COB-ID sin bit 31)
    uint32_t tpdo1_cob_id_enable = (0x180 + node_id_);
    if (!sendSDO(0x1800, 0x01, (uint8_t*)&tpdo1_cob_id_enable, 4, true)) {
        std::cerr << "[" << name_ << "] Error habilitando TPDO1" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Repetir para TPDO2 si es necesario, o si quieres mapear otra cosa
    // Por ahora, solo configuramos TPDO1 para la posición.
    // Si el encoder tiene un segundo TPDO para la misma posición,
    // o para velocidad, etc., habría que mapearlo aquí.
    // Basado en tu descripción original, TPDO1 y TPDO2 eran para "absolute encoder counts".
    // Si son redundantes, puedes configurar TPDO2 de forma similar.
    // Si TPDO2 es para otra cosa (ej. velocidad), el mapeo 0x6000:01 no sería correcto.
    // Por simplicidad, de momento nos centramos en TPDO1.

    std::cout << "[" << name_ << "] TPDOs configurados (0x180 + node_id)" << std::endl;
    return true;
}
// ...

bool EncoderDriver::configureNodeGuard() {
    uint16_t guard_time = 200;
    if (!sendSDO(0x100C, 0x00, (uint8_t*)&guard_time, 2, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint8_t life_time_factor = 15;
    if (!sendSDO(0x100D, 0x00, &life_time_factor, 1, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return true;
}

bool EncoderDriver::startOperational() {
    if (!sendNMT(0x01)) { // START_REMOTE_NODE
        std::cerr << "[" << name_ << "] Error enviando START" << std::endl;
        return false;
    }
    nmt_state_ = NMTState::OPERATIONAL;
    resetTimeout(last_nodeguard_time_);
    resetTimeout(last_heartbeat_time_);
    resetTimeout(last_sample_time_);

    std::cout << "[" << name_ << "] Encoder en estado OPERATIONAL" << std::endl;
    return true;
}

void EncoderDriver::update() {
    if (!isOperational()) return;

    // NodeGuard cada 200 ms
    if (checkTimeout(last_nodeguard_time_, 200)) {
        sendNodeGuard();
        resetTimeout(last_nodeguard_time_);
    }

    // Podrías hacer aquí filtrados o comprobaciones de coherencia
    if (checkTimeout(last_sample_time_, 20)) { // ~50 Hz
        // Filtro simple (ejemplo)
        filtered_position_ = absolute_position_; // aquí puedes poner media o filtro
        resetTimeout(last_sample_time_);
    }
}

// encoder_driver.cpp
void EncoderDriver::processCANFrame(const struct can_frame& frame) {
    uint32_t cob_id = frame.can_id;
    uint32_t id = static_cast<uint32_t>(node_id_);  // ⬅️ Añade esto

    if (cob_id == (0x180 + id)) {  // ⬅️ Usa 'id' en vez de 'node_id_'
        processTPDO1(frame);
    } else if (cob_id == (0x280 + id)) {
        processTPDO2(frame);
    } else if (cob_id == (0x700 + id)) {
        processNodeGuardResponse(frame);
    }
}

void EncoderDriver::processTPDO1(const struct can_frame& frame) {
    if (frame.can_dlc >= 4) {
        absolute_position_ = frame.data[0] |
                             (frame.data[1] << 8) |
                             (frame.data[2] << 16) |
                             (frame.data[3] << 24);
        encoder_ok_ = true;
    }
}

void EncoderDriver::processTPDO2(const struct can_frame& frame) {
    // Si tienes redundancia o segunda lectura, podrías validar aquí
    // De momento simplemente ignoramos o podríamos comprobar coherencia
(void)frame;  // ⬅️ Añade esto para suprimir el warning
}

void EncoderDriver::processNodeGuardResponse(const struct can_frame& frame) {
    if (frame.can_dlc >= 1) {
        communication_ok_ = true;
        resetTimeout(last_heartbeat_time_);
    }
}