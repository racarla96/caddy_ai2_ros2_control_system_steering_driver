#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "caddy_ai2_ros2_control_system_steering_driver/socket_can_interface.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/motor_driver.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/encoder_driver.hpp"

// ============================================================================
// SteeringController — fachada que coordina MotorDriver con la fuente de
// feedback de posición, que puede ser:
//
//   EPC_ENCODER   : encoder CANopen externo (encoder_node_id != 0)
//   POTENTIOMETER : entrada analógica AI1 del propio DZCANTE (encoder_node_id == 0)
//
// Uso:
//   SteeringController sc("can0", 1, 127);  // variante EPC
//   SteeringController sc("can0", 1, 0);    // variante potenciómetro
// ============================================================================

enum class FeedbackSource { EPC_ENCODER, POTENTIOMETER };

class SteeringController
{
public:
    // encoder_node_id == 0  →  variante potenciómetro (AI del DZCANTE)
    // encoder_node_id != 0  →  variante EPC (encoder CANopen externo)
    // analog_pin            →  índice 0-based del canal AI (0=AI1, 1=AI2, 2=AI3)
    //                          ignorado si encoder_node_id != 0
    SteeringController(const std::string& can_iface,
                       uint8_t motor_node_id,
                       uint8_t encoder_node_id,
                       uint8_t analog_pin = 0);

    // Abre el socket CAN y ejecuta las secuencias de arranque.
    // Bloqueante (contiene sleeps internos de configure()).
    bool init();

    // Envía CW_QUICK_STOP al motor y cierra el socket CAN.
    void shutdown();

    // ── Métodos del bucle de control ─────────────────────────────────────────

    // Paso de lectura:
    //   1. Envía SYNC (dispara TPDOs síncronos del drive y del encoder si aplica)
    //   2. Drain no bloqueante del bus CAN
    //   3. Actualiza máquinas de estado
    void cycle_read(double dt);

    // Paso de escritura:
    //   Envía target_position_ al motor por RPDO21 si está habilitado.
    void cycle_write();

    // Almacena la posición target para el próximo cycle_write()
    void set_target_position(int32_t counts);

    // Posición de la fuente de feedback activa:
    //   EPC_ENCODER   → encoder_.get_raw_position() [counts del encoder]
    //   POTENTIOMETER → motor_.get_analog_input_mv() [mV del potenciómetro]
    int32_t get_position() const;

    // Posición del encoder interno del motor (feedback secundario, siempre disponible)
    int32_t get_motor_position() const;

    // True cuando el motor está en OPERATION_ENABLED y la fuente de feedback es válida
    bool is_ready() const;

    // True cuando solo el motor está en OPERATION_ENABLED (sin comprobar el feedback)
    bool is_motor_enabled() const;

    // True cuando el motor tiene NMT==FAULT o DS402==FAULT
    bool has_fault() const;

    // ── Diagnóstico ───────────────────────────────────────────────────────────
    NmtState       get_motor_nmt_state()   const { return motor_.get_nmt_state(); }
    DriveState     get_motor_drive_state() const { return motor_.get_drive_state(); }
    FeedbackSource get_feedback_source()   const { return feedback_source_; }

    // Milivoltios del canal AI activo (siempre disponible si el DZCANTE envía TPDO4)
    int16_t get_analog_input_mv() const { return motor_.get_analog_input_mv(analog_pin_); }

    // ── Para test standalone ──────────────────────────────────────────────────
    void step(double dt);

private:
    void send_sync();
    void receive_frames();

    SocketCANInterface              can_;
    MotorDriver                     motor_;
    std::unique_ptr<EncoderDriver>  encoder_;        // nullptr si POTENTIOMETER
    FeedbackSource                  feedback_source_;
    uint8_t                         analog_pin_;     // canal AI 0-based (solo POT)
    int32_t                         target_position_;
};
