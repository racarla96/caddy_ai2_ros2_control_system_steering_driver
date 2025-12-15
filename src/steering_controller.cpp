#include "caddy_ai2_ros2_control_system_steering_driver/steering_controller.hpp"
#include <iostream>
#include <thread>
#include <vector>
#include <cmath>

SteeringController::SteeringController(const std::string& can_interface_name,
                                       uint8_t motor_node_id,
                                       uint8_t encoder_node_id)
    : can_interface_(can_interface_name)
    , motor_(motor_node_id, "Motor")
    , encoder_(encoder_node_id, "Encoder")
    , running_(false)
    , commanded_position_(0) {
}

bool SteeringController::init() {
    if (!can_interface_.init()) {
        std::cerr << "[SteeringController] Error inicializando SocketCAN" << std::endl;
        return false;
    }

    // Inicializar nodos CANopen
    if (!motor_.initialize(&can_interface_)) {
        std::cerr << "[SteeringController] Error inicializando motor" << std::endl;
        return false;
    }
    if (!encoder_.initialize(&can_interface_)) {
        std::cerr << "[SteeringController] Error inicializando encoder" << std::endl;
        return false;
    }

    // Pasar a OPERATIONAL
    if (!motor_.startOperational()) {
        std::cerr << "[SteeringController] Error arrancando motor" << std::endl;
        return false;
    }
    if (!encoder_.startOperational()) {
        std::cerr << "[SteeringController] Error arrancando encoder" << std::endl;
        return false;
    }

    last_loop_time_ = std::chrono::steady_clock::now();
    last_sync_time_ = last_loop_time_;
    running_ = true;

    std::cout << "[SteeringController] Inicialización completada" << std::endl;
    return true;
}

void SteeringController::run(double loop_hz) {
    const double loop_period_ms = 1000.0 / loop_hz;

    while (running_) {
        auto loop_start = std::chrono::steady_clock::now();

        // 1) Leer y procesar frames CAN
        processIncomingCAN();

        // 2) Llamar a update() de motor y encoder (máquinas de estado + SDO/Nodeguard)
        motor_.update();
        encoder_.update();

        // 3) Enviar SYNC a ~50 Hz (si coincide con loop_hz, simplemente una vez por iteración)
        sendSync();

        // 4) Lógica de control sencilla de steering (ejemplo: ir a commanded_position_)
        if (motor_.isEnabled() && encoder_.isValid()) {
            // Aquí podrías implementar tu control de posición real.
            // De momento, simplemente mandamos la posición target directamente:
            motor_.setTargetPosition(commanded_position_);
        }

        // 5) Esperar hasta completar el periodo deseado
        auto loop_end = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end - loop_start).count();

        if (elapsed_ms < loop_period_ms) {
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(std::round(loop_period_ms - elapsed_ms))));
        }
    }
}

void SteeringController::processIncomingCAN() {
    std::vector<struct can_frame> frames;
    int ret = can_interface_.read(frames, 1); // timeout de 1 ms

    if (ret < 0) {
        // error ya impreso dentro de SocketCANInterface
        return;
    }

    for (const auto& frame : frames) {
        uint32_t cob_id = frame.can_id;
        uint32_t motor_id = static_cast<uint32_t>(motor_.getNodeId());
        uint32_t encoder_id = static_cast<uint32_t>(encoder_.getNodeId());

        // Enrutar a motor o encoder en función del COB-ID
        // Motor: TPDO1 (0x180+id), TPDO21 (0x290+id), TPDO22 (0x2A0+id), SDO (0x580+id), NodeGuard (0x700+id)
        // Encoder: TPDO1 (0x180+id_enc), TPDO2 (0x280+id_enc), NodeGuard (0x700+id_enc)

        if ( (cob_id == (0x180 + motor_id)) ||
             (cob_id == (0x290 + motor_id)) ||
             (cob_id == (0x2A0 + motor_id)) ||
             (cob_id == (0x580 + motor_id)) ||
             (cob_id == (0x700 + motor_id)) ) {

            motor_.processCANFrame(frame);
        }
        else if ( (cob_id == (0x180 + encoder_id)) ||
                  (cob_id == (0x280 + encoder_id)) ||
                  (cob_id == (0x700 + encoder_id)) ) {

            encoder_.processCANFrame(frame);
        }
        // Si hubiera más nodos, se añadirían aquí
    }
}

void SteeringController::sendSync() {
    // Por simplicidad, SYNC a la misma frecuencia que el bucle
    motor_.sendSync();
    // Si el encoder lo necesitase explícitamente, también podrías usar sendSync() desde CANOpenDriver,
    // pero la especificación solo necesita un SYNC global, no uno por nodo.
}

void SteeringController::setTargetSteeringPosition(int32_t position_counts) {
    commanded_position_ = position_counts;
}

// ...
// Elimina la función run() o cámbiala por step()
// void SteeringController::run(double loop_hz) { ... }

void SteeringController::step() {
    // 1) Leer y procesar frames CAN
    processIncomingCAN();

    // 2) Llamar a update() de motor y encoder (máquinas de estado + SDO/Nodeguard)
    motor_.update();
    encoder_.update();

    // 3) Enviar SYNC
    sendSync();

    // 4) Lógica de control sencilla de steering (ejemplo: ir a commanded_position_)
    if (!motor_.isEnabled()) {
        static int c1 = 0;
        if ((c1++ % 100) == 0)
            std::cout << "[Steering] motor not enabled yet\n";
    }
    if (!encoder_.isValid()) {
        static int c2 = 0;
        if ((c2++ % 100) == 0)
            std::cout << "[Steering] encoder not valid yet\n";
    }

    if (motor_.isEnabled() && encoder_.isValid()) {
        motor_.setTargetPosition(commanded_position_);
    }
}

void SteeringController::shutdown() {
    std::cout << "[SteeringController] Apagando controlador (deshabilitando motor)..." << std::endl;

    // Opcional: poner consigna a 0 antes de deshabilitar
    motor_.setTargetPosition(0);

    // Deshabilitar el drive (quita ENABLE_OPERATION)
    motor_.disableMotor();

    // Opcional: si quieres ser más agresivo, puedes añadir un método en MotorDriver
    // para mandar también SHUTDOWN y DISABLE_VOLTAGE por SDO, por ejemplo:
    //
    // motor_.safeStop();   // si lo implementas en MotorDriver
    //
    // Y opcionalmente parar el nodo vía NMT:
    // motor_.stopNode();   // wrapper que haga sendNMT(0x02)
}

// steering_controller.cpp

int32_t SteeringController::getMotorEncoderPosition() const {
    return motor_.getActualPosition();        // o el método equivalente que tengas
}

int32_t SteeringController::getAbsoluteEncoderPosition() const {
    return encoder_.getPositionRaw();         // o getPositionFiltered(), según prefieras
}