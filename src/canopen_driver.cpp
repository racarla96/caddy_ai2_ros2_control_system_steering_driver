#include "caddy_ai2_ros2_control_system_steering_driver/canopen_driver.hpp"

#include <cstring>
#include <iostream>
#include <vector>

using namespace dzcante020l080;

// ── Constructor ──────────────────────────────────────────────────────────────

CANopenDriver::CANopenDriver(SocketCANInterface& can, uint8_t node_id)
    : can_(can)
    , node_id_(node_id)
    , nmt_state_(NmtState::UNKNOWN)
    , nodeguard_elapsed_(0.0)
    , watchdog_elapsed_(0.0)
{}

// ── Lifecycle ────────────────────────────────────────────────────────────────

bool CANopenDriver::configure()
{
    return true;
}

void CANopenDriver::shutdown() {}

// ── Métodos cíclicos (sin sleeps) ────────────────────────────────────────────

void CANopenDriver::update(double dt)
{
    // NodeGuard: enviar RTR periódicamente para mantener la supervisión del drive
    nodeguard_elapsed_ += dt;
    if (nodeguard_elapsed_ >= NODEGUARD_PERIOD_S) {
        send_nodeguard_rtr();
        nodeguard_elapsed_ = 0.0;
    }

    // Watchdog NMT: declarar FAULT si el drive no responde en NMT_TIMEOUT_S
    // reset_watchdog() se llama desde process_nmt_heartbeat() al recibir OPERATIONAL
    watchdog_elapsed_ += dt;
    if (watchdog_elapsed_ >= NMT_TIMEOUT_S) {
        nmt_state_ = NmtState::FAULT;
    }
}

void CANopenDriver::receive_frames()
{
    std::vector<can_frame> frames;
    // timeout_ms = 0: no bloqueante, devuelve inmediatamente los frames disponibles
    can_.read(frames, nullptr, 0);
    for (const auto& frame : frames) {
        process_frame(frame);
    }
}

// ── Primitivas de comunicación ───────────────────────────────────────────────

void CANopenDriver::send_sync()
{
    can_frame frame{};
    frame.can_id  = COB_SYNC;
    frame.can_dlc = 0;
    if (can_.write(frame) != SocketCANInterface::Status::Ok) {
        std::cerr << "[CANopenDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando SYNC\n";
    }
}

bool CANopenDriver::send_nmt(uint8_t command)
{
    can_frame frame{};
    frame.can_id  = COB_NMT_REQUEST;
    frame.can_dlc = 2;
    frame.data[0] = command;
    frame.data[1] = node_id_;
    if (can_.write(frame) != SocketCANInterface::Status::Ok) {
        std::cerr << "[CANopenDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando NMT cmd=0x" << std::hex << static_cast<int>(command)
                  << std::dec << '\n';
        return false;
    }
    return true;
}

bool CANopenDriver::send_sdo_write(uint16_t index, uint8_t subindex,
                                   uint32_t value, uint8_t size)
{
    can_frame frame{};
    frame.can_id  = COB_SDO_REQUEST_BASE + node_id_;
    frame.can_dlc = 8;

    // Byte de comando según número de bytes de datos útiles
    uint8_t cmd;
    switch (size) {
        case 1:  cmd = SDO_CMD_WRITE_1B; break;
        case 2:  cmd = SDO_CMD_WRITE_2B; break;
        default: cmd = SDO_CMD_WRITE_4B; break;  // 4 bytes por defecto
    }

    frame.data[0] = cmd;
    frame.data[1] = static_cast<uint8_t>(index & 0xFFU);
    frame.data[2] = static_cast<uint8_t>((index >> 8) & 0xFFU);
    frame.data[3] = subindex;
    // Datos en little-endian, bytes no usados quedan a 0 (frame{} los inicializa)
    frame.data[4] = static_cast<uint8_t>(value & 0xFFU);
    frame.data[5] = static_cast<uint8_t>((value >>  8) & 0xFFU);
    frame.data[6] = static_cast<uint8_t>((value >> 16) & 0xFFU);
    frame.data[7] = static_cast<uint8_t>((value >> 24) & 0xFFU);

    if (can_.write(frame) != SocketCANInterface::Status::Ok) {
        std::cerr << "[CANopenDriver:" << static_cast<int>(node_id_)
                  << "] Error SDO write idx=0x" << std::hex << index
                  << " sub=0x" << static_cast<int>(subindex) << std::dec << '\n';
        return false;
    }
    return true;
}

bool CANopenDriver::send_sdo_read(uint16_t index, uint8_t subindex)
{
    can_frame frame{};
    frame.can_id  = COB_SDO_REQUEST_BASE + node_id_;
    frame.can_dlc = 8;
    frame.data[0] = SDO_CMD_READ_REQ;   // 0x40 = upload initiate
    frame.data[1] = static_cast<uint8_t>(index & 0xFFU);
    frame.data[2] = static_cast<uint8_t>((index >> 8) & 0xFFU);
    frame.data[3] = subindex;
    // data[4..7] ya son 0 por frame{}
    if (can_.write(frame) != SocketCANInterface::Status::Ok) {
        std::cerr << "[CANopenDriver:" << static_cast<int>(node_id_)
                  << "] Error SDO read idx=0x" << std::hex << index
                  << " sub=0x" << static_cast<int>(subindex) << std::dec << '\n';
        return false;
    }
    return true;
}

bool CANopenDriver::send_pdo(uint16_t cob_id, const uint8_t* data, uint8_t len)
{
    can_frame frame{};
    frame.can_id  = cob_id;
    frame.can_dlc = (len <= 8) ? len : 8;
    if (data != nullptr && frame.can_dlc > 0) {
        std::memcpy(frame.data, data, frame.can_dlc);
    }
    if (can_.write(frame) != SocketCANInterface::Status::Ok) {
        std::cerr << "[CANopenDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando PDO cob=0x" << std::hex << cob_id << std::dec << '\n';
        return false;
    }
    return true;
}

bool CANopenDriver::send_nodeguard_rtr()
{
    can_frame frame{};
    // CAN_RTR_FLAG en can_id indica Remote Transmission Request — crítico para NodeGuard
    frame.can_id  = (COB_NODEGUARD_BASE + node_id_) | CAN_RTR_FLAG;
    frame.can_dlc = 1;  // el drive responde con 1 byte de estado NMT
    if (can_.write(frame) != SocketCANInterface::Status::Ok) {
        std::cerr << "[CANopenDriver:" << static_cast<int>(node_id_)
                  << "] Error enviando NodeGuard RTR\n";
        return false;
    }
    return true;
}

// ── Watchdog y heartbeat ─────────────────────────────────────────────────────

void CANopenDriver::reset_watchdog()
{
    watchdog_elapsed_ = 0.0;
}

void CANopenDriver::process_nmt_heartbeat(uint8_t state_byte)
{
    // Decodifica el byte de estado del NodeGuard response / NMT heartbeat.
    // El toggle-bit (bit 7) alterna en cada respuesta: se manejan ambas variantes.
    switch (state_byte) {
        case NMT_HB_OPERATIONAL_0:
        case NMT_HB_OPERATIONAL_1:
            nmt_state_ = NmtState::OPERATIONAL;
            reset_watchdog();  // drive vivo: reiniciar temporizador de fallo
            break;
        case NMT_HB_STOPPED_0:
        case NMT_HB_STOPPED_1:
            nmt_state_ = NmtState::STOPPED;
            break;
        case NMT_HB_PRE_OPERATIONAL_0:
        case NMT_HB_PRE_OPERATIONAL_1:
            nmt_state_ = NmtState::PRE_OPERATIONAL;
            break;
        default:
            // Byte desconocido: ignorar sin cambiar estado
            break;
    }
}
