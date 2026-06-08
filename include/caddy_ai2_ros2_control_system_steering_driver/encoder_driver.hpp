#pragma once

#include "caddy_ai2_ros2_control_system_steering_driver/canopen_driver.hpp"
#include <cstdint>

// ============================================================================
// EncoderDriver — encoder absoluto CAN externo (EpcEncoder)
//
// Usa el mapping ESTÁNDAR de CANopen (COB_ENC_TPDO1_BASE = 0x180 + node_id),
// a diferencia del motor AMC que usa COB-IDs propietarios.
//
// configure() es bloqueante. El resto es no bloqueante.
// ============================================================================
class EncoderDriver : public CANopenDriver
{
public:
    explicit EncoderDriver(SocketCANInterface& can, uint8_t node_id);

    // Secuencia de arranque: NMT reset → TPDO config → NodeGuard → NMT START.
    // Bloqueante (contiene sleeps).
    bool configure() override;

    // Posición absoluta cruda en counts (int32, little-endian del TPDO1)
    int32_t get_raw_position() const { return raw_position_; }

    // True cuando se ha recibido al menos un TPDO1 válido (≥ 4 bytes)
    bool is_valid() const { return valid_; }

private:
    // ── Configuración (solo desde configure()) ───────────────────────────────
    bool configure_tpdo();       // SDO 0x1800:01 (COB-ID) y 0x1800:02 (trans type)
    bool configure_nodeguard();  // SDO 0x100C (guard_time) y 0x100D (life_factor)

    // ── Procesado de frames ──────────────────────────────────────────────────
    void process_frame(const can_frame& frame) override;
    void process_tpdo1(const can_frame& frame);  // posición absoluta 4 bytes LE

    // ── Estado interno ───────────────────────────────────────────────────────
    int32_t raw_position_;  // último valor recibido del encoder
    bool    valid_;         // false hasta recibir el primer TPDO1 válido
};
