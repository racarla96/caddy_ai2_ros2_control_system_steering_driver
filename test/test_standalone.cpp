// Test sencillo — monitoriza el estado del motor sin seguimiento de referencia.
//
// Uso:
//   ./test_standalone <can> [motor_id] [encoder_id]
//
// Inicializa el drive, espera OPERATION_ENABLED y se queda en posición 0
// imprimiendo motor_pos, enc_pos y estado cada segundo. Ctrl+C para salir.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

#include "caddy_ai2_ros2_control_system_steering_driver/steering_controller.hpp"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static const char* nmt_to_str(NMTState s)
{
    switch (s) {
        case NMTState::BOOT_UP:         return "BOOT_UP";
        case NMTState::PRE_OPERATIONAL: return "PRE_OP";
        case NMTState::OPERATIONAL:     return "OPERATIONAL";
        case NMTState::STOPPED:         return "STOPPED";
        default:                        return "?";
    }
}

static const char* motor_to_str(MotorState s)
{
    switch (s) {
        case MotorState::NOT_READY_TO_SWITCH_ON: return "NOT_RDY";
        case MotorState::SWITCH_ON_DISABLED:     return "SOD";
        case MotorState::READY_TO_SWITCH_ON:     return "RTSO";
        case MotorState::SWITCHED_ON:            return "SW_ON";
        case MotorState::OPERATION_ENABLED:      return "OP_ENA";
        case MotorState::QUICK_STOP_ACTIVE:      return "QS";
        case MotorState::FAULT_REACTION_ACTIVE:  return "FAULT_RXN";
        case MotorState::FAULT:                  return "FAULT";
        default:                                 return "?";
    }
}

int main(int argc, char* argv[])
{
    const std::string can_iface  = (argc > 1) ? argv[1] : "vcan0";
    const uint8_t motor_id   = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    const uint8_t encoder_id = (argc > 3) ? static_cast<uint8_t>(std::atoi(argv[3])) : 127;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Steering Driver — Test Simple ===\n"
              << "  CAN        : " << can_iface << "\n"
              << "  motor_id   : " << static_cast<int>(motor_id) << "\n"
              << "  encoder_id : " << static_cast<int>(encoder_id) << "\n"
              << "  Ctrl+C para salir\n\n";

    SteeringController sc(can_iface, motor_id, encoder_id);

    std::cout << "[init] Abriendo CAN y arrancando drive...\n";
    if (!sc.init()) {
        std::cerr << "[ERROR] init() falló.\n";
        return 1;
    }
    std::cout << "[init] OK\n\n";

    constexpr double DT           = 0.02;   // 50 Hz
    constexpr double WAIT_TIMEOUT = 10.0;
    constexpr int    PRINT_EVERY  = 5;      // cada 100 ms en espera

    std::cout << "[espera] Aguardando OPERATION_ENABLED (máx " << WAIT_TIMEOUT << " s)...\n";

    double wait_elapsed = 0.0;
    int    wait_iter    = 0;

    while (g_running && !sc.isMotorEnabled() && wait_elapsed < WAIT_TIMEOUT) {
        const auto t0 = std::chrono::steady_clock::now();
        sc.step();
        wait_elapsed += DT;
        ++wait_iter;

        if (wait_iter % PRINT_EVERY == 0) {
            std::cout << "  t=" << std::fixed << std::setprecision(1) << wait_elapsed
                      << "s  NMT=" << nmt_to_str(sc.getMotorNMTState())
                      << "  Motor=" << motor_to_str(sc.getMotorState())
                      << "\n";
        }

        if (sc.hasFault()) {
            std::cerr << "[ERROR] FAULT durante la espera — abortando\n";
            sc.shutdown();
            return 1;
        }

        const auto elapsed   = std::chrono::steady_clock::now() - t0;
        const auto remaining = std::chrono::milliseconds(static_cast<int>(DT * 1000)) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    if (!sc.isMotorEnabled()) {
        std::cerr << "[ERROR] Timeout — el drive no llegó a OPERATION_ENABLED\n";
        sc.shutdown();
        return 1;
    }
    std::cout << "[espera] Listo\n\n";

    std::cout << std::left
              << std::setw(8)  << "t_s"
              << std::setw(12) << "motor_pos"
              << std::setw(12) << "enc_pos"
              << std::setw(14) << "NMT"
              << std::setw(10) << "Motor"
              << "\n" << std::string(56, '-') << "\n";

    sc.setTargetSteeringPosition(0);

    double elapsed = 0.0;
    int    iter    = 0;

    while (g_running) {
        const auto t0 = std::chrono::steady_clock::now();
        sc.step();
        elapsed += DT;
        ++iter;

        if (iter % 50 == 0) {  // cada segundo
            std::cout << std::left
                      << std::setw(8)  << std::fixed << std::setprecision(1) << elapsed
                      << std::setw(12) << sc.getMotorEncoderPosition()
                      << std::setw(12) << sc.getAbsoluteEncoderPosition()
                      << std::setw(14) << nmt_to_str(sc.getMotorNMTState())
                      << std::setw(10) << motor_to_str(sc.getMotorState())
                      << "\n";
        }

        if (sc.hasFault()) {
            std::cerr << "\n[ERROR] FAULT detectado\n";
            break;
        }

        const auto loop_elapsed = std::chrono::steady_clock::now() - t0;
        const auto remaining = std::chrono::milliseconds(static_cast<int>(DT * 1000)) - loop_elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    std::cout << "\n[shutdown]\n";
    sc.shutdown();
    return 0;
}
