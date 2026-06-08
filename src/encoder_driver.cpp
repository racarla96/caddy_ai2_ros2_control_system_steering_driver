#include "caddy_ai2_ros2_control_system_steering_driver/encoder_driver.hpp"

#include <chrono>
#include <iostream>
#include <thread>

using namespace dzcante020l080;

// ── Constructor ──────────────────────────────────────────────────────────────

EncoderDriver::EncoderDriver(SocketCANInterface& can, uint8_t node_id)
    : CANopenDriver(can, node_id)
    , raw_position_(0)
    , valid_(false)
{}

// ── Lifecycle ────────────────────────────────────────────────────────────────

bool EncoderDriver::configure()
{
    std::cout << "[EncoderDriver:" << static_cast<int>(node_id_)
              << "] Iniciando configuración...\n";

    // 1. Reset de comunicación
    if (!send_nmt(NMT_RESET_COMMUNICATION)) {
        std::cerr << "[EncoderDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando NMT RESET_COMM\n";
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_RESET_MS));

    // 2. Entrar en PRE_OPERATIONAL para configurar vía SDO
    if (!send_nmt(NMT_ENTER_PRE_OPERATIONAL)) {
        std::cerr << "[EncoderDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando NMT PRE_OP\n";
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PREOP_MS));
    nmt_state_ = NmtState::PRE_OPERATIONAL;

    // 3. Configurar TPDO1 (posición absoluta)
    if (!configure_tpdo()) {
        std::cerr << "[EncoderDriver:" << static_cast<int>(node_id_)
                  << "] Error configurando TPDO1\n";
        return false;
    }

    // 4. Configurar NodeGuard
    if (!configure_nodeguard()) {
        std::cerr << "[EncoderDriver:" << static_cast<int>(node_id_)
                  << "] Error configurando NodeGuard\n";
        return false;
    }

    // 5. NMT START → el encoder pasa a OPERATIONAL y empieza a enviar TPDOs
    if (!send_nmt(NMT_START_REMOTE_NODE)) {
        std::cerr << "[EncoderDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando NMT START\n";
        return false;
    }

    nmt_state_ = NmtState::OPERATIONAL;
    reset_watchdog();  // inicia el contador de 2 s para recibir el primer heartbeat

    std::cout << "[EncoderDriver:" << static_cast<int>(node_id_)
              << "] Configuración completada. NMT=OPERATIONAL\n";
    return true;
}

// ── Configuración (bloqueante, solo en configure()) ──────────────────────────

bool EncoderDriver::configure_tpdo()
{
    // TPDO1 — posición absoluta (encoder→master, cada SYNC)
    // COB-ID = 0x180 + node_id  (standard CANopen TPDO1, a diferencia del motor AMC)
    if (!send_sdo_write(SDO_IDX_TPDO1_COMM, SDO_SUB_PDO_COB_ID,
                        COB_ENC_TPDO1_BASE + node_id_, 4)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    // Tipo de transmisión: 1 = síncrono (cada SYNC dispara el TPDO)
    if (!send_sdo_write(SDO_IDX_TPDO1_COMM, SDO_SUB_PDO_TRANSMISSION_TYPE,
                        ENC_TPDO1_TRANSMISSION_TYPE, 1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    return true;
}

bool EncoderDriver::configure_nodeguard()
{
    // Guard time en milisegundos
    if (!send_sdo_write(SDO_IDX_GUARD_TIME, SDO_SUB_GUARD_TIME,
                        NODEGUARD_GUARD_TIME_MS, 2)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    // Life time factor: timeout = guard_time × life_factor ms
    if (!send_sdo_write(SDO_IDX_LIFE_FACTOR, SDO_SUB_LIFE_FACTOR,
                        NODEGUARD_LIFE_FACTOR, 1)) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(STARTUP_SLEEP_PDO_MS));

    return true;
}

// ── Procesado de frames ──────────────────────────────────────────────────────

void EncoderDriver::process_frame(const can_frame& frame)
{
    // Eliminar el RTR flag para la comparación de COB-ID
    const uint32_t bare_id = frame.can_id & ~static_cast<uint32_t>(CAN_RTR_FLAG);
    const uint32_t nid = node_id_;

    if (bare_id == (COB_ENC_TPDO1_BASE + nid)) {
        process_tpdo1(frame);
    } else if (bare_id == (COB_NODEGUARD_BASE + nid)) {
        // Solo procesar la respuesta del encoder (sin RTR flag)
        if (!(frame.can_id & CAN_RTR_FLAG) && frame.can_dlc >= 1) {
            process_nmt_heartbeat(frame.data[0]);
        }
    }
}

void EncoderDriver::process_tpdo1(const can_frame& frame)
{
    if (frame.can_dlc < 4) return;

    raw_position_ = static_cast<int32_t>(
        static_cast<uint32_t>(frame.data[0])         |
        (static_cast<uint32_t>(frame.data[1]) <<  8) |
        (static_cast<uint32_t>(frame.data[2]) << 16) |
        (static_cast<uint32_t>(frame.data[3]) << 24));

    valid_ = true;  // primer TPDO1 válido recibido
}
