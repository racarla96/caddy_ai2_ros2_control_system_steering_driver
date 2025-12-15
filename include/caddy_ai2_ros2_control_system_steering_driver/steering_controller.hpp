// steering_controller.hpp
#ifndef STEERING_CONTROLLER_HPP
#define STEERING_CONTROLLER_HPP

#include "socket_can_interface.hpp"
#include "motor_driver.hpp"
#include "encoder_driver.hpp"
#include <memory>

class SteeringController {
public:
    SteeringController(const std::string& can_interface_name,
                       uint8_t motor_node_id,
                       uint8_t encoder_node_id);

    bool init();
    void run(double loop_hz = 50.0);   // bucle principal 1 thread
    void stop() { running_ = false; }
    void step(); // Nueva función para una sola iteración
    void shutdown();

    // Interface sencilla para comandos
    void setTargetSteeringPosition(int32_t position_counts);

// Posición del encoder del drive (MotorDriver)
    int32_t getMotorEncoderPosition() const;

    // Posición del encoder absoluto externo (EncoderDriver)
    int32_t getAbsoluteEncoderPosition() const;

private:
    void processIncomingCAN();  // leer frames y enrutar a motor/encoder
    void sendSync();            // enviar SYNC a 50 Hz aproximadamente

    SocketCANInterface can_interface_;
    MotorDriver motor_;
    EncoderDriver encoder_;

    bool running_;
    std::chrono::steady_clock::time_point last_loop_time_;
    std::chrono::steady_clock::time_point last_sync_time_;

    int32_t commanded_position_;
};

#endif // STEERING_CONTROLLER_HPP