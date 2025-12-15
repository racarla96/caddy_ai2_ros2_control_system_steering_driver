// encoder_driver.hpp
#ifndef ENCODER_DRIVER_HPP
#define ENCODER_DRIVER_HPP

#include "canopen_driver.hpp"
#include <cstdint>
#include <thread>
#include <chrono>

class EncoderDriver : public CANOpenDriver {
public:
    EncoderDriver(uint8_t node_id, const std::string& name = "Encoder");
    virtual ~EncoderDriver() = default;

    // Implementación de métodos virtuales
    bool initialize(SocketCANInterface* can_interface) override;
    bool startOperational() override;
    void update() override;
    void processCANFrame(const struct can_frame& frame) override;

    // Getters
    int32_t getAbsolutePosition() const { return absolute_position_; }
    int32_t getFilteredPosition() const { return filtered_position_; }

// Nuevos métodos públicos
    bool isValid() const { return encoder_ok_; }
    int32_t getPositionRaw() const { return absolute_position_; }
    int32_t getPositionFiltered() const { return filtered_position_; }

private:
    bool configureTPDOs();
    bool configureNodeGuard();

    void processTPDO1(const struct can_frame& frame); // posición absoluta 1
    void processTPDO2(const struct can_frame& frame); // posición absoluta 2 / redundante
    void processNodeGuardResponse(const struct can_frame& frame);

    int32_t absolute_position_;
    int32_t filtered_position_;
    bool encoder_ok_;

    std::chrono::steady_clock::time_point last_sample_time_;
};

#endif // ENCODER_DRIVER_HPP