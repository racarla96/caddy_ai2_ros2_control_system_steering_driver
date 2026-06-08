#include "caddy_ai2_ros2_control_system_steering_driver/steering_controller.hpp"

#include <iostream>
#include <vector>

// ── Constructor ──────────────────────────────────────────────────────────────

SteeringController::SteeringController(const std::string& can_iface,
                                       uint8_t motor_node_id,
                                       uint8_t encoder_node_id,
                                       uint8_t analog_pin)
    : can_(can_iface)
    , motor_(can_, motor_node_id)
    , encoder_(encoder_node_id != 0
               ? std::make_unique<EncoderDriver>(can_, encoder_node_id)
               : nullptr)
    , feedback_source_(encoder_node_id != 0
                       ? FeedbackSource::EPC_ENCODER
                       : FeedbackSource::POTENTIOMETER)
    , analog_pin_(analog_pin < 3 ? analog_pin : 0)
    , target_position_(0)
{}

// ── Lifecycle ────────────────────────────────────────────────────────────────

bool SteeringController::init()
{
    if (!can_.init()) {
        std::cerr << "[SteeringController] Error inicializando SocketCAN\n";
        return false;
    }

    if (!motor_.configure()) {
        std::cerr << "[SteeringController] Error configurando motor\n";
        can_.close();
        return false;
    }

    if (encoder_) {
        if (!encoder_->configure()) {
            std::cerr << "[SteeringController] Error configurando encoder EPC\n";
            can_.close();
            return false;
        }
        std::cout << "[SteeringController] Modo: EPC_ENCODER\n";
    } else {
        std::cout << "[SteeringController] Modo: POTENTIOMETER (AI"
                  << static_cast<int>(analog_pin_ + 1) << " del DZCANTE, pin idx "
                  << static_cast<int>(analog_pin_) << ")\n";
    }

    std::cout << "[SteeringController] Inicialización completada\n";
    return true;
}

void SteeringController::shutdown()
{
    motor_.shutdown();  // envía CW_QUICK_STOP inmediatamente vía RPDO1
    can_.close();
}

// ── Bucle de control ─────────────────────────────────────────────────────────

void SteeringController::cycle_read(double dt)
{
    // 1. En modo potenciómetro: pedir lectura analógica por SDO (0x201A).
    //    La respuesta llega antes de la siguiente llamada a receive_frames().
    if (feedback_source_ == FeedbackSource::POTENTIOMETER) {
        motor_.poll_analog_input(analog_pin_);
    }

    // 2. SYNC: dispara los TPDOs síncronos del drive (y del encoder EPC si aplica).
    send_sync();

    // 3. Drain no bloqueante: recoge todos los frames y los distribuye.
    receive_frames();

    // 4. Actualizar máquinas de estado de motor (y encoder si aplica).
    motor_.update(dt);
    if (encoder_) {
        encoder_->update(dt);
    }
}

void SteeringController::cycle_write()
{
    if (motor_.is_operation_enabled()) {
        motor_.set_target_position(target_position_);
    }
}

// ── API pública ──────────────────────────────────────────────────────────────

void SteeringController::set_target_position(int32_t counts)
{
    target_position_ = counts;
}

int32_t SteeringController::get_position() const
{
    if (feedback_source_ == FeedbackSource::POTENTIOMETER) {
        return static_cast<int32_t>(motor_.get_analog_input_mv(analog_pin_));
    }
    return encoder_->get_raw_position();
}

int32_t SteeringController::get_motor_position() const
{
    return motor_.get_actual_position();
}

bool SteeringController::is_ready() const
{
    if (feedback_source_ == FeedbackSource::POTENTIOMETER) {
        return motor_.is_operation_enabled() && motor_.is_analog_input_valid();
    }
    return motor_.is_operation_enabled() && encoder_->is_valid();
}

bool SteeringController::is_motor_enabled() const
{
    return motor_.is_operation_enabled();
}

bool SteeringController::has_fault() const
{
    return motor_.get_nmt_state() == NmtState::FAULT ||
           motor_.get_drive_state() == DriveState::FAULT;
}

void SteeringController::step(double dt)
{
    cycle_read(dt);
    cycle_write();
}

// ── Privados ─────────────────────────────────────────────────────────────────

void SteeringController::send_sync()
{
    motor_.send_sync();
}

void SteeringController::receive_frames()
{
    std::vector<can_frame> frames;
    can_.read(frames, nullptr, 0);

    for (const auto& frame : frames) {
        motor_.dispatch_frame(frame);
        if (encoder_) {
            encoder_->dispatch_frame(frame);
        }
    }
}
