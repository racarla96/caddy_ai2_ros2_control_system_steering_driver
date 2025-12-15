// canopen_driver.hpp
#ifndef CANOPEN_DRIVER_HPP
#define CANOPEN_DRIVER_HPP

#include <linux/can.h>
#include <cstdint>
#include <string>
#include <chrono>

class SocketCANInterface;

// Estados NMT de CANopen
enum class NMTState {
    BOOT_UP,
    STOPPED,
    OPERATIONAL,
    PRE_OPERATIONAL
};

// Clase base para dispositivos CANopen
class CANOpenDriver {
public:
    CANOpenDriver(uint8_t node_id, const std::string& name);
    virtual ~CANOpenDriver() = default;

    // Métodos virtuales puros que deben implementar las clases derivadas
    virtual bool initialize(SocketCANInterface* can_interface) = 0;
    virtual bool startOperational() = 0;
    virtual void update() = 0;  // Llamado en cada ciclo del loop principal
    virtual void processCANFrame(const struct can_frame& frame) = 0;

    // Getters
    uint8_t getNodeId() const { return node_id_; }
    const std::string& getName() const { return name_; }
    NMTState getNMTState() const { return nmt_state_; }
    bool isOperational() const { return nmt_state_ == NMTState::OPERATIONAL; }

    bool sendSync();

protected:
    // Métodos auxiliares para comunicación CANopen
    bool sendNMT(uint8_t command);
    bool sendSDO(uint16_t index, uint8_t subindex, const uint8_t* data, uint8_t data_len, bool write = true);
    bool sendPDO(uint16_t cob_id, const uint8_t* data, uint8_t data_len);
    bool sendNodeGuard();

    // Utilidades para construcción de mensajes
    void buildCANFrame(struct can_frame& frame, uint32_t can_id, const uint8_t* data, uint8_t len);
    
    // Gestión de timeouts
    bool checkTimeout(std::chrono::steady_clock::time_point& last_time, int timeout_ms);
    void resetTimeout(std::chrono::steady_clock::time_point& time_point);

    // Miembros protegidos
    uint8_t node_id_;
    std::string name_;
    NMTState nmt_state_;
    SocketCANInterface* can_interface_;
    
    // Timeouts
    std::chrono::steady_clock::time_point last_nodeguard_time_;
    std::chrono::steady_clock::time_point last_heartbeat_time_;
    bool communication_ok_;
};

#endif // CANOPEN_DRIVER_HPP