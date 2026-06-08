#pragma once

#include "caddy_ai2_ros2_control_system_steering_driver/canopen_driver.hpp"
#include <cstdint>

// Estados DS402 del drive (CiA 402 Profile Position)
enum class DriveState {
    UNKNOWN,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    OPERATION_DISABLED,   // también llamado "Switched On" en la norma
    OPERATION_ENABLED,
    QUICK_STOP,
    FAULT
};

// ============================================================================
// MotorDriver — driver CiA 402 en modo Profile Position para DZCANTE-020L080
//
// configure() es bloqueante (con sleeps). El resto de métodos públicos son
// no bloqueantes y seguros de llamar desde el bucle de control.
// ============================================================================
class MotorDriver : public CANopenDriver
{
public:
    explicit MotorDriver(SocketCANInterface& can, uint8_t node_id);

    // Secuencia completa de arranque: NMT reset → PDO config → modo posición
    // → ControlWord SDO sequence → NMT START. Bloqueante.
    bool configure() override;

    // Envía CW_QUICK_STOP inmediatamente vía RPDO1 antes de cerrar el socket.
    void shutdown() override;

    // Bucle cíclico: llama a CANopenDriver::update(dt) y luego a tick_ds402(dt).
    // Sin sleeps.
    void update(double dt) override;

    // Envía la posición target por RPDO21 (ASYNC, sin esperar SYNC)
    void set_target_position(int32_t counts);

    // Establece el estado DS402 deseado. tick_ds402() hará las transiciones.
    void request_state(DriveState desired);

    DriveState get_drive_state()    const { return drive_state_; }
    bool       is_operation_enabled() const { return drive_state_ == DriveState::OPERATION_ENABLED; }
    int32_t    get_actual_position() const { return actual_position_; }

    // Entradas analógicas del DZCANTE (AI1-AI3), en milivoltios.
    // Leídas por SDO polling a objeto AMC propietario 0x201A (subindex 1-based).
    // Llamar a poll_analog_input(pin) en cada ciclo para solicitar la lectura;
    // la respuesta llega en el ciclo siguiente vía process_sdo_response().
    // pin: índice 0-based (0=AI1, 1=AI2, 2=AI3)
    void    poll_analog_input(uint8_t pin);
    int16_t get_analog_input_mv(uint8_t pin = 0) const {
        return (pin < 3) ? analog_inputs_mv_[pin] : 0;
    }
    bool    is_analog_input_valid()  const { return analog_input_valid_; }

private:
    // ── Métodos de configuración (solo llamados desde configure()) ───────────
    bool configure_pdos();       // SDOs para COB-IDs y tipos de transmisión
    bool configure_nodeguard();  // SDOs 0x100C (guard_time) y 0x100D (life_factor)
    bool configure_mode();       // SDO 0x6060:00 = Profile Position (0x01)
    bool startup_sequence();     // CW DISABLE_VOLTAGE→SHUTDOWN→SWITCH_ON→ENABLE_OP
                                 // vía SDO + NMT START

    // ── Máquina de estados DS402 (solo se llama desde update()) ─────────────
    // Solo actúa si NMT==OPERATIONAL y ha pasado DS402_MIN_INTERVAL_S desde
    // el último ControlWord enviado.
    void tick_ds402(double dt);

    // ── Procesado de frames CAN ──────────────────────────────────────────────
    void process_frame(const can_frame& frame) override;
    void process_tpdo1(const can_frame& frame);       // StatusWord → update_motor_state()
    void process_tpdo21(const can_frame& frame);      // ActualPosition (int32 LE)
    void process_sdo_response(const can_frame& frame);// SDO upload response (0x201A → AI)

    // ── Decodificación del StatusWord (DS402) ────────────────────────────────
    // Usa exactamente las máscaras del driver original rbcar:
    //   (sw & 0x4F): SWITCH_ON_DISABLED y FAULT
    //   (sw & 0x6F): resto de estados
    void update_motor_state();

    // ── Envío del ControlWord ────────────────────────────────────────────────
    void send_control_word_sdo(uint16_t cw);   // vía SDO — solo durante startup (PRE_OP)
    void send_control_word_pdo(uint16_t cw);   // vía RPDO1 — durante operación (OPERATIONAL)

    // ── Estado interno ───────────────────────────────────────────────────────
    DriveState drive_state_;       // estado actual decodificado del StatusWord
    DriveState desired_state_;     // estado que tick_ds402() intentará alcanzar
    uint16_t   status_word_;       // último StatusWord recibido por TPDO1
    int32_t    actual_position_;   // último ActualPosition recibido por TPDO21
    double     ds402_interval_;    // tiempo acumulado entre ControlWords (s)
    int16_t    analog_inputs_mv_[3]{}; // AI1, AI2, AI3 del DZCANTE en mV
    bool       analog_input_valid_;   // true tras recibir el primer TPDO de AI
};
