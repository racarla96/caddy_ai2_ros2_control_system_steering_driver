#include "caddy_ai2_ros2_control_system_steering_driver/motor_driver.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

using namespace dzcante020l080;

// ── Constructor ──────────────────────────────────────────────────────────────

MotorDriver::MotorDriver(SocketCANInterface& can, uint8_t node_id)
    : CANopenDriver(can, node_id)
    , drive_state_(DriveState::UNKNOWN)
    , desired_state_(DriveState::OPERATION_ENABLED)
    , status_word_(0)
    , actual_position_(0)
    , ds402_interval_(0.0)
    , analog_inputs_mv_{}
    , analog_input_valid_(false)
{}

// ── Lifecycle ────────────────────────────────────────────────────────────────

bool MotorDriver::configure()
{
    std::cout << "[MotorDriver:" << static_cast<int>(node_id_)
              << "] Iniciando configuración...\n";

    // 1. Reset de comunicación → el drive vuelve al estado por defecto
    if (!send_nmt(NMT_RESET_COMMUNICATION)) {
        std::cerr << "[MotorDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando NMT RESET_COMM\n";
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_RESET_MS));

    // 2. Entrar en PRE_OPERATIONAL para configurar PDOs vía SDO
    if (!send_nmt(NMT_ENTER_PRE_OPERATIONAL)) {
        std::cerr << "[MotorDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando NMT PRE_OP\n";
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PREOP_MS));
    nmt_state_ = NmtState::PRE_OPERATIONAL;

    // 3. Configurar PDOs (COB-IDs y tipos de transmisión)
    if (!configure_pdos()) {
        std::cerr << "[MotorDriver:" << static_cast<int>(node_id_)
                  << "] Error configurando PDOs\n";
        return false;
    }

    // 4. Configurar NodeGuard
    if (!configure_nodeguard()) {
        std::cerr << "[MotorDriver:" << static_cast<int>(node_id_)
                  << "] Error configurando NodeGuard\n";
        return false;
    }

    // 5. Configurar modo de operación (Profile Position)
    if (!configure_mode()) {
        std::cerr << "[MotorDriver:" << static_cast<int>(node_id_)
                  << "] Error configurando modo posición\n";
        return false;
    }

    // 6+7. Secuencia ControlWord + NMT START → OPERATIONAL
    if (!startup_sequence()) {
        std::cerr << "[MotorDriver:" << static_cast<int>(node_id_)
                  << "] Error en secuencia de arranque\n";
        return false;
    }

    std::cout << "[MotorDriver:" << static_cast<int>(node_id_)
              << "] Configuración completada. NMT=OPERATIONAL\n";
    return true;
}

void MotorDriver::shutdown()
{
    // Envío inmediato de QUICK_STOP antes de que el socket se cierre
    send_control_word_pdo(CW_QUICK_STOP);
}

// ── Métodos cíclicos ─────────────────────────────────────────────────────────

void MotorDriver::update(double dt)
{
    CANopenDriver::update(dt);  // watchdog NMT + NodeGuard RTR periódico
    tick_ds402(dt);             // transiciones de estado DS402 (solo si OPERATIONAL)
}

// ── API pública ──────────────────────────────────────────────────────────────

void MotorDriver::poll_analog_input(uint8_t pin)
{
    if (pin >= 3) return;
    // SDO upload request → 0x201A:subindex (1-based)
    send_sdo_read(SDO_IDX_ANALOG_INPUT, static_cast<uint8_t>(pin + 1));
}

void MotorDriver::set_target_position(int32_t counts)
{
    // RPDO21 — ASYNC: el drive acepta la posición inmediatamente, sin esperar SYNC
    const uint8_t data[4] = {
        static_cast<uint8_t>(counts & 0xFF),
        static_cast<uint8_t>((counts >>  8) & 0xFF),
        static_cast<uint8_t>((counts >> 16) & 0xFF),
        static_cast<uint8_t>((counts >> 24) & 0xFF)
    };
    send_pdo(static_cast<uint16_t>(COB_RPDO21_BASE + node_id_), data, 4);
}

void MotorDriver::request_state(DriveState desired)
{
    desired_state_ = desired;
}

// ── Configuración (bloqueante, solo en configure()) ──────────────────────────

bool MotorDriver::configure_pdos()
{
    // RPDO1 — ControlWord (master→drive, ASYNC)
    // COB-ID: 0x180 + node_id  (AMC propietario — NO es el 0x200 estándar)
    if (!send_sdo_write(SDO_IDX_RPDO1_COMM, SDO_SUB_PDO_COB_ID,
                        COB_RPDO1_BASE + node_id_, 4)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    if (!send_sdo_write(SDO_IDX_RPDO1_COMM, SDO_SUB_PDO_TRANSMISSION_TYPE,
                        RPDO_TRANSMISSION_TYPE_ASYNC, 1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    // RPDO21 — TargetPosition (master→drive, ASYNC)
    // COB-ID: 0x280 + node_id  (AMC propietario — NO es el 0x300/0x400 estándar)
    if (!send_sdo_write(SDO_IDX_RPDO21_COMM, SDO_SUB_PDO_COB_ID,
                        COB_RPDO21_BASE + node_id_, 4)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    if (!send_sdo_write(SDO_IDX_RPDO21_COMM, SDO_SUB_PDO_TRANSMISSION_TYPE,
                        RPDO_TRANSMISSION_TYPE_ASYNC, 1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    // TPDO1 — StatusWord (drive→master, cada 10 SYNCs)
    // COB-ID: 0x4A0 + node_id  (AMC propietario — NO es el 0x180 estándar)
    if (!send_sdo_write(SDO_IDX_TPDO1_COMM, SDO_SUB_PDO_COB_ID,
                        COB_TPDO1_BASE + node_id_, 4)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    if (!send_sdo_write(SDO_IDX_TPDO1_COMM, SDO_SUB_PDO_TRANSMISSION_TYPE,
                        TPDO1_TRANSMISSION_TYPE, 1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    // TPDO21 — ActualPosition (drive→master, cada SYNC)
    // COB-ID: 0x400 + node_id  (AMC propietario — NO es el 0x280 estándar)
    if (!send_sdo_write(SDO_IDX_TPDO21_COMM, SDO_SUB_PDO_COB_ID,
                        COB_TPDO21_BASE + node_id_, 4)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    if (!send_sdo_write(SDO_IDX_TPDO21_COMM, SDO_SUB_PDO_TRANSMISSION_TYPE,
                        TPDO21_TRANSMISSION_TYPE, 1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    return true;
}

bool MotorDriver::configure_nodeguard()
{
    // Guard time en milisegundos: el drive produce un heartbeat cada guard_time ms
    if (!send_sdo_write(SDO_IDX_GUARD_TIME, SDO_SUB_GUARD_TIME,
                        NODEGUARD_GUARD_TIME_MS, 2)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    // Life time factor: timeout = guard_time * life_factor ms
    if (!send_sdo_write(SDO_IDX_LIFE_FACTOR, SDO_SUB_LIFE_FACTOR,
                        NODEGUARD_LIFE_FACTOR, 1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    return true;
}

bool MotorDriver::configure_mode()
{
    // Modo de operación: 0x01 = Profile Position Mode
    if (!send_sdo_write(SDO_IDX_MODES_OF_OP, SDO_SUB_MODES_OF_OP,
                        DRIVE_MODE_POSITION, 1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));
    return true;
}

bool MotorDriver::startup_sequence()
{
    // Secuencia de ControlWords vía SDO (drive en PRE_OPERATIONAL)
    // Lleva al drive desde SWITCH_ON_DISABLED hasta OPERATION_ENABLED antes
    // de pasar a OPERATIONAL, evitando que el drive entre en OPERATIONAL
    // sin un estado de potencia definido.

    send_control_word_sdo(CW_DISABLE_VOLTAGE);
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_SDO_MS));

    send_control_word_sdo(CW_SHUTDOWN);
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_SDO_MS));

    send_control_word_sdo(CW_SWITCH_ON);
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_SDO_MS));

    send_control_word_sdo(CW_ENABLE_OP);
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_SDO_MS));

    // NMT START: el drive pasa a OPERATIONAL y comienza a enviar TPDOs síncronos
    if (!send_nmt(NMT_START_REMOTE_NODE)) {
        std::cerr << "[MotorDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando NMT START\n";
        return false;
    }

    // Estado optimista: el watchdog detectará si el drive no confirma
    nmt_state_ = NmtState::OPERATIONAL;
    reset_watchdog();  // inicia el contador de 2 s para recibir el primer heartbeat
    return true;
}

// ── Máquina de estados DS402 ─────────────────────────────────────────────────

void MotorDriver::tick_ds402(double dt)
{
    if (nmt_state_ != NmtState::OPERATIONAL) return;

    ds402_interval_ += dt;
    if (ds402_interval_ < DS402_MIN_INTERVAL_S) return;
    ds402_interval_ = 0.0;

    // Recuperación de fallo antes de cualquier transición
    if (drive_state_ == DriveState::FAULT) {
        send_control_word_pdo(CW_FAULT_RESET);
        return;
    }

    if (desired_state_ == DriveState::OPERATION_ENABLED) {
        switch (drive_state_) {
            case DriveState::SWITCH_ON_DISABLED:
                // → READY_TO_SWITCH_ON
                send_control_word_pdo(CW_SHUTDOWN);
                break;
            case DriveState::READY_TO_SWITCH_ON:
                // → OPERATION_DISABLED (Switched On)
                send_control_word_pdo(CW_SWITCH_ON);
                break;
            case DriveState::OPERATION_DISABLED:
                // → OPERATION_ENABLED
                send_control_word_pdo(CW_ENABLE_OP);
                break;
            case DriveState::OPERATION_ENABLED:
                // Mantener estado enviando ENABLE_OP periódicamente
                send_control_word_pdo(CW_ENABLE_OP);
                break;
            case DriveState::QUICK_STOP:
                // Salir de QUICK_STOP volviendo a SWITCH_ON_DISABLED
                send_control_word_pdo(CW_DISABLE_VOLTAGE);
                break;
            default:
                // UNKNOWN: esperando el primer StatusWord del TPDO1
                break;
        }
    } else if (desired_state_ == DriveState::QUICK_STOP) {
        send_control_word_pdo(CW_QUICK_STOP);
    }
}

// ── Procesado de frames ──────────────────────────────────────────────────────

void MotorDriver::process_frame(const can_frame& frame)
{
    // Eliminar el RTR flag para la comparación de COB-ID
    const uint32_t bare_id = frame.can_id & ~static_cast<uint32_t>(CAN_RTR_FLAG);
    const uint32_t nid = node_id_;

    if (bare_id == (COB_TPDO1_BASE + nid)) {
        process_tpdo1(frame);
    } else if (bare_id == (COB_TPDO21_BASE + nid)) {
        process_tpdo21(frame);
    } else if (bare_id == (COB_SDO_RESPONSE_BASE + nid)) {
        process_sdo_response(frame);
    } else if (bare_id == (COB_NODEGUARD_BASE + nid)) {
        // Solo procesar la respuesta del drive (sin RTR flag), no el eco de nuestra RTR
        if (!(frame.can_id & CAN_RTR_FLAG) && frame.can_dlc >= 1) {
            process_nmt_heartbeat(frame.data[0]);
        }
    }
}

void MotorDriver::process_tpdo1(const can_frame& frame)
{
    if (frame.can_dlc < 2) return;
    status_word_ = static_cast<uint16_t>(frame.data[0]) |
                   (static_cast<uint16_t>(frame.data[1]) << 8);
    update_motor_state();
}

void MotorDriver::process_tpdo21(const can_frame& frame)
{
    if (frame.can_dlc < 4) return;
    actual_position_ = static_cast<int32_t>(
        static_cast<uint32_t>(frame.data[0])        |
        (static_cast<uint32_t>(frame.data[1]) <<  8) |
        (static_cast<uint32_t>(frame.data[2]) << 16) |
        (static_cast<uint32_t>(frame.data[3]) << 24));
}

void MotorDriver::process_sdo_response(const can_frame& frame)
{
    if (frame.can_dlc < 4) return;

    const uint8_t  cmd   = frame.data[0];
    const uint16_t idx   = static_cast<uint16_t>(frame.data[1]) |
                           (static_cast<uint16_t>(frame.data[2]) << 8);
    const uint8_t  sub   = frame.data[3];

    if (cmd == 0x80) {
        // SDO abort
        std::cerr << "[MotorDriver:" << static_cast<int>(node_id_)
                  << "] SDO abort idx=0x" << std::hex << idx
                  << " sub=0x" << static_cast<int>(sub) << std::dec << '\n';
        return;
    }

    // SDO upload response (0x4B=2B, 0x43=4B, 0x4F=1B): bits 7:5 == 0b010
    if ((cmd & 0xE0) == 0x40 && idx == SDO_IDX_ANALOG_INPUT) {
        // 0x201A:N → entrada analógica AI(N), subindex 1-based
        if (sub >= 1 && sub <= 3 && frame.can_dlc >= 6) {
            const int16_t raw = static_cast<int16_t>(
                static_cast<uint16_t>(frame.data[4]) |
                (static_cast<uint16_t>(frame.data[5]) << 8));
            // raw * 20V / 16384 = volts → ×1000 → mV
            analog_inputs_mv_[sub - 1] = static_cast<int16_t>(
                static_cast<int32_t>(raw) * ANALOG_INPUT_MV_NUM / ANALOG_INPUT_MV_DEN);
            analog_input_valid_ = true;
        }
    }
    // 0x60xx = SDO download response (write confirmado) — ignorar
}

// ── Decodificación del StatusWord ────────────────────────────────────────────

void MotorDriver::update_motor_state()
{
    const uint16_t sw = status_word_;

    // Máscaras idénticas a las del driver original rbcar (verificadas contra CiA 402)
    if      ((sw & SW_MASK_SOD)   == SW_SWITCH_ON_DISABLED) drive_state_ = DriveState::SWITCH_ON_DISABLED;
    else if ((sw & SW_MASK_STATE) == SW_READY_TO_SWITCH_ON)  drive_state_ = DriveState::READY_TO_SWITCH_ON;
    else if ((sw & SW_MASK_STATE) == SW_OPERATION_DISABLED)  drive_state_ = DriveState::OPERATION_DISABLED;
    else if ((sw & SW_MASK_STATE) == SW_OPERATION_ENABLED)   drive_state_ = DriveState::OPERATION_ENABLED;
    else if ((sw & SW_MASK_STATE) == SW_QUICK_STOP_ACTIVE)   drive_state_ = DriveState::QUICK_STOP;
    else if ((sw & SW_MASK_SOD)   == SW_FAULT)               drive_state_ = DriveState::FAULT;
    // Si ninguna máscara coincide: drive_state_ no cambia (puede estar en UNKNOWN
    // durante los primeros ciclos antes de recibir el TPDO1)
}

// ── ControlWord helpers ──────────────────────────────────────────────────────

void MotorDriver::send_control_word_sdo(uint16_t cw)
{
    // Usado solo durante startup (PRE_OPERATIONAL): el drive acepta SDOs
    send_sdo_write(SDO_IDX_CONTROL_WORD, SDO_SUB_CONTROL_WORD,
                   static_cast<uint32_t>(cw), 2);
}

void MotorDriver::send_control_word_pdo(uint16_t cw)
{
    // Usado durante operación (OPERATIONAL): RPDO1 ASYNC
    const uint8_t data[2] = {
        static_cast<uint8_t>(cw & 0xFF),
        static_cast<uint8_t>((cw >> 8) & 0xFF)
    };
    send_pdo(static_cast<uint16_t>(COB_RPDO1_BASE + node_id_), data, 2);
}
