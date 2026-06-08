#pragma once

#include <cstdint>
#include <linux/can.h>

#include "caddy_ai2_ros2_control_system_steering_driver/socket_can_interface.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/dzcante020l080_constants.hpp"

// Estados NMT del nodo CANopen
enum class NmtState {
    UNKNOWN,
    PRE_OPERATIONAL,
    OPERATIONAL,
    STOPPED,
    FAULT
};

// ============================================================================
// CANopenDriver — clase base para dispositivos CANopen
//
// Gestiona el watchdog NMT y el NodeGuard periódico.
// No contiene lógica de ROS, ni hilos, ni sleeps en métodos cíclicos.
// Los sleeps están reservados para configure() / shutdown().
// ============================================================================
class CANopenDriver
{
public:
    explicit CANopenDriver(SocketCANInterface& can, uint8_t node_id);
    virtual ~CANopenDriver() = default;

    // Secuencia de arranque bloqueante (solo se llama desde on_configure, nunca
    // desde el bucle de control). Las subclases sobreescriben este método.
    virtual bool configure();

    // Apagado del nodo. Las subclases sobreescriben si necesitan acciones extra.
    virtual void shutdown();

    // Bucle cíclico no bloqueante: actualiza watchdog NMT + NodeGuard periódico.
    // NO contiene sleeps. Se llama desde cycle_read() de SteeringController.
    virtual void update(double dt);

    // Drain no bloqueante del bus CAN. Pasa cada frame a process_frame().
    // Se usa en modo standalone; en SteeringController el routing lo hace el SC.
    void receive_frames();

    // Envía un frame SYNC (COB-ID 0x080, DLC=0).
    void send_sync();

    NmtState get_nmt_state() const { return nmt_state_; }
    bool     is_communication_ok() const { return nmt_state_ == NmtState::OPERATIONAL; }
    uint8_t  get_node_id() const { return node_id_; }

    // Punto de entrada público para que SteeringController enrute frames a este driver.
    // Cada subclase filtra internamente por su node_id en process_frame() (protected).
    void dispatch_frame(const can_frame& frame) { process_frame(frame); }

protected:
    // ── Primitivas de comunicación ──────────────────────────────────────────

    // Envía un comando NMT (frame 0x000, 2 bytes: [command, node_id])
    bool send_nmt(uint8_t command);

    // SDO expedited download (write al drive). size: 1, 2 o 4 bytes.
    // Los bytes no usados de data[4..7] se rellenan con 0.
    bool send_sdo_write(uint16_t index, uint8_t subindex, uint32_t value, uint8_t size);

    // SDO upload initiate (read del drive). La respuesta llega en el siguiente
    // drain de frames y debe procesarse en process_sdo_response() de la subclase.
    bool send_sdo_read(uint16_t index, uint8_t subindex);

    // Envía un PDO (frame CAN arbitrario con los datos indicados)
    bool send_pdo(uint16_t cob_id, const uint8_t* data, uint8_t len);

    // NodeGuard RTR: frame con can_id = (0x700+node_id) | CAN_RTR_FLAG
    // El drive responde con su estado NMT en un byte (process_nmt_heartbeat)
    bool send_nodeguard_rtr();

    // Reinicia el contador del watchdog NMT. Se llama desde process_nmt_heartbeat
    // cada vez que el drive confirma estar en OPERATIONAL.
    void reset_watchdog();

    // Decodifica el byte de estado del heartbeat / NodeGuard response y actualiza
    // nmt_state_. Llama a reset_watchdog() cuando el drive está OPERATIONAL.
    void process_nmt_heartbeat(uint8_t state_byte);

    // Procesa un frame CAN recibido. Las subclases filtran por su node_id.
    // Deben llamar a process_nmt_heartbeat() cuando detecten COB_NODEGUARD_BASE+node_id_.
    virtual void process_frame(const can_frame& frame) = 0;

    // ── Miembros compartidos con subclases ─────────────────────────────────
    SocketCANInterface& can_;
    uint8_t   node_id_;
    NmtState  nmt_state_;

private:
    double nodeguard_elapsed_;   // segundos desde el último NodeGuard RTR
    double watchdog_elapsed_;    // segundos desde el último heartbeat OPERATIONAL
};
