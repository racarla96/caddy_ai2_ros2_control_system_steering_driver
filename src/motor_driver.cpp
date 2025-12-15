#include "caddy_ai2_ros2_control_system_steering_driver/motor_driver.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/socket_can_interface.hpp"
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>

// Máscaras para decodificar el Status Word (CiA 402)
#define SW_READY_TO_SWITCH_ON   0x01
#define SW_SWITCHED_ON          0x02
#define SW_OPERATION_ENABLED    0x04
#define SW_FAULT                0x08
#define SW_VOLTAGE_ENABLED      0x10
#define SW_QUICK_STOP           0x20
#define SW_SWITCH_ON_DISABLED   0x40
#define SW_WARNING              0x80

MotorDriver::MotorDriver(uint8_t node_id, const std::string& name)
    : CANOpenDriver(node_id, name)
    , motor_state_(MotorState::NOT_READY_TO_SWITCH_ON)
    , status_word_(0)
    , control_word_(0)
    , target_position_(0)
    , actual_position_(0)
    , actual_velocity_(0)
    , actual_current_(0)
    , dc_bus_voltage_(0)
    , sdo_read_counter_(0) {
}

bool MotorDriver::initialize(SocketCANInterface* can_interface) {
    can_interface_ = can_interface;
    
    std::cout << "[" << name_ << "] Iniciando configuración..." << std::endl;

    // Reset de comunicación
    if (!sendNMT(0x82)) {  // NMT_RESET_COMMUNICATION
        std::cerr << "[" << name_ << "] Error enviando RESET_COMMUNICATION" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Entrar en Pre-Operational
    if (!sendNMT(0x80)) {  // NMT_ENTER_PRE_OPERATIONAL
        std::cerr << "[" << name_ << "] Error enviando PRE_OPERATIONAL" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    nmt_state_ = NMTState::PRE_OPERATIONAL;

    // Configurar PDOs
    if (!configureRPDOs()) {
        std::cerr << "[" << name_ << "] Error configurando RPDOs" << std::endl;
        return false;
    }

    if (!configureTPDOs()) {
        std::cerr << "[" << name_ << "] Error configurando TPDOs" << std::endl;
        return false;
    }

    // Configurar Node Guard
    if (!configureNodeGuard()) {
        std::cerr << "[" << name_ << "] Error configurando Node Guard" << std::endl;
        return false;
    }

    // Configurar modo de operación
    if (!configureDriveMode()) {
        std::cerr << "[" << name_ << "] Error configurando modo de operación" << std::endl;
        return false;
    }

    std::cout << "[" << name_ << "] Configuración completada" << std::endl;
    return true;
}

bool MotorDriver::configureRPDOs() {
    // Configuración de RPDO 1 (Control Word) - COB-ID 0x200 + node_id
    uint32_t rpdo1_cob_id = 0x200 + node_id_;
    if (!sendSDO(0x1400, 0x01, (uint8_t*)&rpdo1_cob_id, 4, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Configuración de RPDO 21 (Target Position) - COB-ID 0x230 + node_id
    uint32_t rpdo21_cob_id = 0x230 + node_id_;
    if (!sendSDO(0x1414, 0x01, (uint8_t*)&rpdo21_cob_id, 4, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Configuración de RPDO 22 (Target Velocity) - COB-ID 0x240 + node_id
    uint32_t rpdo22_cob_id = 0x240 + node_id_;
    if (!sendSDO(0x1415, 0x01, (uint8_t*)&rpdo22_cob_id, 4, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return true;
}

bool MotorDriver::configureTPDOs() {
    // Configuración de TPDO 1 (Status Word) - COB-ID 0x180 + node_id
    uint32_t tpdo1_cob_id = 0x180 + node_id_;
    if (!sendSDO(0x1800, 0x01, (uint8_t*)&tpdo1_cob_id, 4, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Configuración de TPDO 21 (Actual Position) - COB-ID 0x290 + node_id
    uint32_t tpdo21_cob_id = 0x290 + node_id_;
    if (!sendSDO(0x1814, 0x01, (uint8_t*)&tpdo21_cob_id, 4, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Configuración de TPDO 22 (Actual Velocity) - COB-ID 0x2A0 + node_id
    uint32_t tpdo22_cob_id = 0x2A0 + node_id_;
    if (!sendSDO(0x1815, 0x01, (uint8_t*)&tpdo22_cob_id, 4, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return true;
}

bool MotorDriver::configureNodeGuard() {
    // Guard Time (ms)
    uint16_t guard_time = 200;  // 200ms
    if (!sendSDO(0x100C, 0x00, (uint8_t*)&guard_time, 2, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Life Time Factor
    uint8_t life_time_factor = 15;
    if (!sendSDO(0x100D, 0x00, &life_time_factor, 1, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return true;
}

bool MotorDriver::configureDriveMode() {
    // Configurar modo Profile Position (0x01)
    uint8_t mode = 0x01;
    if (!sendSDO(0x6060, 0x00, &mode, 1, true)) {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return true;
}

// --- NUEVO: mandar ControlWord por SDO (para secuencia de arranque) ---
bool MotorDriver::sendControlWordSDO(uint16_t control_word) {
    control_word_ = control_word;

    uint8_t data[2];
    data[0] = control_word & 0xFF;
    data[1] = (control_word >> 8) & 0xFF;

    std::cout << "[" << name_ << "] Enviando ControlWord por SDO = 0x"
              << std::hex << control_word << std::dec << std::endl;

    // 0x6040: ControlWord, subíndice 0x00
    return sendSDO(0x6040, 0x00, data, 2, true);
}

bool MotorDriver::startOperational() {
    std::cout << "[" << name_ << "] Iniciando secuencia de arranque del motor..." << std::endl;

    // Secuencia de power-up del motor por SDO (en PRE-OPERATIONAL), como en el driver original

    // 1. Disable Voltage (opcional, dejar a 0)
    if (!sendControlWordSDO(static_cast<uint16_t>(MotorCommand::DISABLE_VOLTAGE))) {
        std::cerr << "[" << name_ << "] Error enviando DISABLE_VOLTAGE (SDO)" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 2. Shutdown
    if (!sendControlWordSDO(static_cast<uint16_t>(MotorCommand::SHUTDOWN))) {
        std::cerr << "[" << name_ << "] Error enviando SHUTDOWN (SDO)" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 3. Switch On
    if (!sendControlWordSDO(static_cast<uint16_t>(MotorCommand::SWITCH_ON))) {
        std::cerr << "[" << name_ << "] Error enviando SWITCH_ON (SDO)" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 4. Enable Operation
    if (!sendControlWordSDO(static_cast<uint16_t>(MotorCommand::ENABLE_OPERATION))) {
        std::cerr << "[" << name_ << "] Error enviando ENABLE_OPERATION (SDO)" << std::endl;
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Ahora sí, cambiar a estado OPERATIONAL
    if (!sendNMT(0x01)) {  // NMT_START_REMOTE_NODE
        std::cerr << "[" << name_ << "] Error enviando START" << std::endl;
        return false;
    }

    nmt_state_ = NMTState::OPERATIONAL;
    resetTimeout(last_nodeguard_time_);
    resetTimeout(last_sdo_read_time_);

    std::cout << "[" << name_ << "] Motor en estado OPERATIONAL" << std::endl;
    return true;
}

void MotorDriver::update() {
    static int sync_counter = 0;
    static auto last_print = std::chrono::steady_clock::now();

    if (!isOperational()) return;

    // Actualizar estado del motor basado en status word
    updateMotorState();

    // NodeGuard a 5 Hz
    if (checkTimeout(last_nodeguard_time_, 200)) {
        sendNodeGuard();
        resetTimeout(last_nodeguard_time_);
    }

    // SDO reads a 5 Hz
    if (checkTimeout(last_sdo_read_time_, 200)) {
        // ... (rotación SDO)
        sdo_read_counter_++;
        resetTimeout(last_sdo_read_time_);
    }

    // DEBUG: medir frecuencia real de llamadas a update() (debería ~50 Hz)
    sync_counter++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count();
    if (elapsed >= 1) {
        std::cout << "[MotorDriver] update() calls per second: " << sync_counter << std::endl;
        sync_counter = 0;
        last_print = now;
    }
}

void MotorDriver::processCANFrame(const struct can_frame& frame) {
    uint32_t cob_id = frame.can_id;
    uint32_t id = static_cast<uint32_t>(node_id_);

    // TPDO 1: Status Word (0x180 + node_id)
    if (cob_id == (0x180 + id)) {
        processTPDO1(frame);
    }
    // TPDO 21: Actual Position (0x290 + node_id)
    else if (cob_id == (0x290 + id)) {
        processTPDO21(frame);
    }
    // TPDO 22: Actual Velocity (0x2A0 + node_id)
    else if (cob_id == (0x2A0 + id)) {
        processTPDO22(frame);
    }
    // SDO Response (0x580 + node_id)
    else if (cob_id == (0x580 + id)) {
        processSDOResponse(frame);
    }
    // Node Guard Response (0x700 + node_id)
    else if (cob_id == (0x700 + id)) {
        processNodeGuardResponse(frame);
    }
}

void MotorDriver::processTPDO1(const struct can_frame& frame) {
    if (frame.can_dlc >= 2) {
        uint16_t new_sw = frame.data[0] | (frame.data[1] << 8);

        if (new_sw != status_word_) {
            status_word_ = new_sw;

            std::cout << "[MotorDriver] StatusWord = 0x"
                      << std::hex << status_word_ << std::dec << " (";

            if (status_word_ & SW_READY_TO_SWITCH_ON)   std::cout << "RDY ";
            if (status_word_ & SW_SWITCHED_ON)          std::cout << "SWO ";
            if (status_word_ & SW_OPERATION_ENABLED)    std::cout << "OPE ";
            if (status_word_ & SW_FAULT)                std::cout << "FLT ";
            if (status_word_ & SW_VOLTAGE_ENABLED)      std::cout << "VEN ";
            if (status_word_ & SW_QUICK_STOP)           std::cout << "QST ";
            if (status_word_ & SW_SWITCH_ON_DISABLED)   std::cout << "SWOD ";
            if (status_word_ & SW_WARNING)              std::cout << "WRN ";

            std::cout << ")" << std::endl;
        }
    }
}

void MotorDriver::processTPDO21(const struct can_frame& frame) {
    if (frame.can_dlc >= 4) {
        actual_position_ = frame.data[0] | (frame.data[1] << 8) | 
                          (frame.data[2] << 16) | (frame.data[3] << 24);
    }
}

void MotorDriver::processTPDO22(const struct can_frame& frame) {
    if (frame.can_dlc >= 4) {
        actual_velocity_ = frame.data[0] | (frame.data[1] << 8) | 
                          (frame.data[2] << 16) | (frame.data[3] << 24);
    }
}

void MotorDriver::processSDOResponse(const struct can_frame& frame) {
    if (frame.can_dlc < 8) return;

    uint8_t command = frame.data[0];
    uint16_t index = frame.data[1] | (frame.data[2] << 8);
    uint8_t subindex = frame.data[3];

    // SDO Upload Response
    if ((command & 0xE0) == 0x40) {
        if (index == 0x6078 && subindex == 0x00) {
            // Corriente actual
            actual_current_ = frame.data[4] | (frame.data[5] << 8);
        } else if (index == 0x6079 && subindex == 0x00) {
            // Voltaje DC Bus
            dc_bus_voltage_ = frame.data[4] | (frame.data[5] << 8);
        }
    }
}

void MotorDriver::processNodeGuardResponse(const struct can_frame& frame) {
    if (frame.can_dlc >= 1) {
        communication_ok_ = true;
        resetTimeout(last_heartbeat_time_);
    }
}

void MotorDriver::updateMotorState() {
    uint16_t sw = status_word_;

    // Decodificar estado según CiA 402
    if ((sw & 0x4F) == 0x00) {
        motor_state_ = MotorState::NOT_READY_TO_SWITCH_ON;
    } else if ((sw & 0x4F) == 0x40) {
        motor_state_ = MotorState::SWITCH_ON_DISABLED;
    } else if ((sw & 0x6F) == 0x21) {
        motor_state_ = MotorState::READY_TO_SWITCH_ON;
    } else if ((sw & 0x6F) == 0x23) {
        motor_state_ = MotorState::SWITCHED_ON;
    } else if ((sw & 0x6F) == 0x27) {
        motor_state_ = MotorState::OPERATION_ENABLED;
    } else if ((sw & 0x6F) == 0x07) {
        motor_state_ = MotorState::QUICK_STOP_ACTIVE;
    } else if ((sw & 0x4F) == 0x0F) {
        motor_state_ = MotorState::FAULT_REACTION_ACTIVE;
    } else if ((sw & 0x4F) == 0x08) {
        motor_state_ = MotorState::FAULT;
    }
}

bool MotorDriver::sendControlWord(uint16_t control_word) {
    // Versión PDO: usar en modo cíclico, cuando el drive ya está en OPERATIONAL
    control_word_ = control_word;
    uint8_t data[2];
    data[0] = control_word & 0xFF;
    data[1] = (control_word >> 8) & 0xFF;

    return sendPDO(0x200 + node_id_, data, 2);
}

bool MotorDriver::setTargetPosition(int32_t position) {
    target_position_ = position;
    uint8_t data[4];
    data[0] = position & 0xFF;
    data[1] = (position >> 8) & 0xFF;
    data[2] = (position >> 16) & 0xFF;
    data[3] = (position >> 24) & 0xFF;

    bool ok = sendPDO(0x230 + node_id_, data, 4);
    if (!ok) {
        std::cerr << "[MotorDriver] ERROR sending target position PDO, pos=" << position << std::endl;
    } else {
        // DEBUG: imprime algunas veces
        static int c = 0;
        if ((c++ % 50) == 0) {
            std::cout << "[MotorDriver] Sent target position PDO: " << position << std::endl;
        }
    }
    return ok;
}

bool MotorDriver::setTargetVelocity(int32_t velocity) {
    uint8_t data[4];
    data[0] = velocity & 0xFF;
    data[1] = (velocity >> 8) & 0xFF;
    data[2] = (velocity >> 16) & 0xFF;
    data[3] = (velocity >> 24) & 0xFF;
    
    return sendPDO(0x240 + node_id_, data, 4);
}

bool MotorDriver::enableMotor() {
    return sendControlWord(static_cast<uint16_t>(MotorCommand::ENABLE_OPERATION));
}

bool MotorDriver::disableMotor() {
    return sendControlWord(static_cast<uint16_t>(MotorCommand::DISABLE_OPERATION));
}

bool MotorDriver::resetFault() {
    return sendControlWord(static_cast<uint16_t>(MotorCommand::FAULT_RESET));
}

// motor_driver.cpp
// ...
bool MotorDriver::isEnabled() const {
    // Operation enabled (bit 2) debe estar a 1 (0x0004)
    // Fault (bit 3) debe estar a 0 (0x0008)
    // NO verificamos Quick Stop (bit 5) porque puede estar activo en modo normal
    return (status_word_ & 0x0004) && !(status_word_ & 0x0008);
}
// ...