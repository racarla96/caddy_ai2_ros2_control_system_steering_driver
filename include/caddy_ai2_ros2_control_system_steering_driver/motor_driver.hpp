// motor_driver.hpp
#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "canopen_driver.hpp"
#include <cstdint>
#include <thread>
#include <chrono>

// Estados del motor según CiA 402
enum class MotorState {
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE,
    FAULT
};

// Comandos de control del motor
enum class MotorCommand {
    SHUTDOWN = 0x06,
    SWITCH_ON = 0x07,
    DISABLE_VOLTAGE = 0x00,
    QUICK_STOP = 0x02,
    DISABLE_OPERATION = 0x07,
    ENABLE_OPERATION = 0x0F,
    FAULT_RESET = 0x80
};

class MotorDriver : public CANOpenDriver {
public:
    MotorDriver(uint8_t node_id, const std::string& name = "Motor");
    virtual ~MotorDriver() = default;

    // Implementación de métodos virtuales
    bool initialize(SocketCANInterface* can_interface) override;
    bool startOperational() override;
    void update() override;
    void processCANFrame(const struct can_frame& frame) override;

    // Métodos específicos del motor
    bool setTargetPosition(int32_t position);
    bool setTargetVelocity(int32_t velocity);
    bool enableMotor();
    bool disableMotor();
    bool resetFault();

    // Getters de estado
    MotorState getMotorState() const { return motor_state_; }
    int32_t getActualPosition() const { return actual_position_; }
    int32_t getActualVelocity() const { return actual_velocity_; }
    uint16_t getStatusWord() const { return status_word_; }
    int16_t getActualCurrent() const { return actual_current_; }
    uint16_t getDCBusVoltage() const { return dc_bus_voltage_; }

    bool isEnabled() const; // Asegúrate de que sea const y override

    bool sendControlWordSDO(uint16_t control_word);

private:
    // Configuración inicial del motor
    bool configureRPDOs();
    bool configureTPDOs();
    bool configureNodeGuard();
    bool configureDriveMode();
    
    // Gestión de estados
    void updateMotorState();
    bool sendControlWord(uint16_t control_word);
    
    // Procesamiento de PDOs
    void processTPDO1(const struct can_frame& frame);  // Status Word
    void processTPDO21(const struct can_frame& frame); // Actual Position
    void processTPDO22(const struct can_frame& frame); // Actual Velocity
    void processSDOResponse(const struct can_frame& frame);
    void processNodeGuardResponse(const struct can_frame& frame);

    // Variables de estado del motor
    MotorState motor_state_;
    uint16_t status_word_;
    uint16_t control_word_;
    int32_t target_position_;
    int32_t actual_position_;
    int32_t actual_velocity_;
    int16_t actual_current_;
    uint16_t dc_bus_voltage_;
    
    // Control de lectura SDO
    std::chrono::steady_clock::time_point last_sdo_read_time_;
    int sdo_read_counter_;
};

#endif // MOTOR_DRIVER_HPP