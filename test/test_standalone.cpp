// Test sencillo — monitoriza el estado del motor, sin seguimiento de referencia.
//
// Uso:
//   ./test_standalone <can> [motor_id]
//
// Inicializa el drive, espera OPERATION_ENABLED y se queda en posición 0
// imprimiendo el estado cada segundo. Ctrl+C para salir.

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

static const char* nmt_to_str(NmtState s)
{
    switch (s) {
        case NmtState::UNKNOWN:         return "UNKNOWN";
        case NmtState::PRE_OPERATIONAL: return "PRE_OP";
        case NmtState::OPERATIONAL:     return "OPERATIONAL";
        case NmtState::STOPPED:         return "STOPPED";
        case NmtState::FAULT:           return "FAULT";
        default:                        return "?";
    }
}

static const char* drive_to_str(DriveState s)
{
    switch (s) {
        case DriveState::UNKNOWN:            return "UNKNOWN";
        case DriveState::SWITCH_ON_DISABLED: return "SOD";
        case DriveState::READY_TO_SWITCH_ON: return "RTSO";
        case DriveState::OPERATION_DISABLED: return "OP_DIS";
        case DriveState::OPERATION_ENABLED:  return "OP_ENA";
        case DriveState::QUICK_STOP:         return "QS";
        case DriveState::FAULT:              return "FAULT";
        default:                             return "?";
    }
}

int main(int argc, char* argv[])
{
    const std::string can_iface = (argc > 1) ? argv[1] : "vcan0";
    const uint8_t motor_id      = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Steering Driver — Test Simple ===\n"
              << "  CAN       : " << can_iface << "\n"
              << "  motor_id  : " << static_cast<int>(motor_id) << "\n"
              << "  Ctrl+C para salir\n\n";

    // Modo EPC con encoder_id=0 activa POTENTIOMETER internamente, pero en este
    // test no usamos feedback analógico — solo monitorizamos el estado DS402.
    SteeringController sc(can_iface, motor_id, 0 /*no encoder*/, 2 /*pin*/);

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

    while (g_running && !sc.is_motor_enabled() && wait_elapsed < WAIT_TIMEOUT) {
        const auto t0 = std::chrono::steady_clock::now();
        sc.cycle_read(DT);
        wait_elapsed += DT;
        ++wait_iter;

        if (wait_iter % PRINT_EVERY == 0) {
            std::cout << "  t=" << std::fixed << std::setprecision(1) << wait_elapsed
                      << "s  NMT=" << nmt_to_str(sc.get_motor_nmt_state())
                      << "  DS402=" << drive_to_str(sc.get_motor_drive_state())
                      << "\n";
        }

        if (sc.has_fault()) {
            std::cerr << "[ERROR] FAULT durante la espera — abortando\n";
            sc.shutdown();
            return 1;
        }

        const auto elapsed   = std::chrono::steady_clock::now() - t0;
        const auto remaining = std::chrono::milliseconds(static_cast<int>(DT * 1000)) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    if (!sc.is_motor_enabled()) {
        std::cerr << "[ERROR] Timeout — el drive no llegó a OPERATION_ENABLED\n";
        sc.shutdown();
        return 1;
    }
    std::cout << "[espera] Listo\n\n";

    std::cout << std::left
              << std::setw(8)  << "t_s"
              << std::setw(12) << "motor_pos"
              << std::setw(8)  << "pot_mV"
              << std::setw(14) << "NMT"
              << std::setw(10) << "DS402"
              << "\n" << std::string(52, '-') << "\n";

    sc.set_target_position(0);

    double elapsed = 0.0;
    int    iter    = 0;

    while (g_running) {
        const auto t0 = std::chrono::steady_clock::now();
        sc.step(DT);
        elapsed += DT;
        ++iter;

        if (iter % 50 == 0) {  // cada segundo
            std::cout << std::left
                      << std::setw(8)  << std::fixed << std::setprecision(1) << elapsed
                      << std::setw(12) << sc.get_motor_position()
                      << std::setw(8)  << sc.get_analog_input_mv()
                      << std::setw(14) << nmt_to_str(sc.get_motor_nmt_state())
                      << std::setw(10) << drive_to_str(sc.get_motor_drive_state())
                      << "\n";
        }

        if (sc.has_fault()) {
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
