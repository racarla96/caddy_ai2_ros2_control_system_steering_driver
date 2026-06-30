// Test senoidal — seguimiento de referencia sinusoidal con encoder EPC.
//
// Uso:
//   ./test_sinusoidal <can> [motor_id] [encoder_id] [min] [max] [freq_hz] [cycles]
//
// Ejemplos:
//   ./test_sinusoidal can0 1 127 -97500 99100 0.05 3

#include <atomic>
#include <chrono>
#include <cmath>
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
    const std::string can_iface = (argc > 1) ? argv[1] : "vcan0";
    const uint8_t  motor_id   = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    const uint8_t  encoder_id = (argc > 3) ? static_cast<uint8_t>(std::atoi(argv[3])) : 127;
    const int32_t  min_counts = (argc > 4) ? static_cast<int32_t>(std::atol(argv[4])) : -10000;
    const int32_t  max_counts = (argc > 5) ? static_cast<int32_t>(std::atol(argv[5])) :  10000;
    const double   freq_hz    = (argc > 6) ? std::atof(argv[6]) : 0.1;
    const double   num_cycles = (argc > 7) ? std::atof(argv[7]) : 3.0;

    if (min_counts >= max_counts) {
        std::cerr << "[ERROR] min_counts debe ser < max_counts\n";
        return 1;
    }

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    const double center    = 0.5 * (min_counts + max_counts);
    const double amplitude = 0.5 * (max_counts - min_counts);
    const double period_s  = 1.0 / freq_hz;
    const double total_s   = num_cycles * period_s;

    std::cout << "=== Steering Driver — Test Senoidal ===\n"
              << "  CAN          : " << can_iface << "\n"
              << "  motor_id     : " << static_cast<int>(motor_id) << "\n"
              << "  encoder_id   : " << static_cast<int>(encoder_id) << "\n"
              << "  Rango motor  : [" << min_counts << ", " << max_counts << "] counts\n"
              << "  Sinusoide    : " << freq_hz << " Hz × " << num_cycles
              << " ciclos = " << total_s << " s\n"
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
    constexpr int    PRINT_EVERY  = 5;

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
                      << "  fault=" << (sc.hasFault() ? "SI" : "no") << "\n";
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
              << std::setw(7)  << "t_s"
              << std::setw(10) << "ref"
              << std::setw(12) << "motor_pos"
              << std::setw(10) << "error"
              << std::setw(12) << "enc_pos"
              << std::setw(14) << "NMT"
              << std::setw(10) << "Motor"
              << "\n" << std::string(75, '-') << "\n";

    constexpr int PRINT_EVERY_TEST = 10;  // cada 200 ms

    double test_elapsed = 0.0;
    int    test_iter    = 0;

    while (g_running && test_elapsed < total_s) {
        const auto t0 = std::chrono::steady_clock::now();

        const double  ref_f = center + amplitude * std::sin(2.0 * M_PI * freq_hz * test_elapsed);
        const int32_t ref   = static_cast<int32_t>(std::round(ref_f));

        sc.setTargetSteeringPosition(ref);
        sc.step();

        ++test_iter;
        test_elapsed += DT;

        if (test_iter % PRINT_EVERY_TEST == 0) {
            const int32_t motor_pos = sc.getMotorEncoderPosition();
            const int32_t enc_pos   = sc.getAbsoluteEncoderPosition();
            const int32_t error     = ref - motor_pos;

            std::cout << std::left
                      << std::setw(7)  << std::fixed << std::setprecision(2) << test_elapsed
                      << std::setw(10) << ref
                      << std::setw(12) << motor_pos
                      << std::setw(10) << error
                      << std::setw(12) << enc_pos
                      << std::setw(14) << nmt_to_str(sc.getMotorNMTState())
                      << std::setw(10) << motor_to_str(sc.getMotorState())
                      << "\n";
        }

        if (sc.hasFault()) {
            std::cerr << "\n[ERROR] FAULT detectado — abortando\n";
            break;
        }

        const auto elapsed   = std::chrono::steady_clock::now() - t0;
        const auto remaining = std::chrono::milliseconds(static_cast<int>(DT * 1000)) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    std::cout << "\n[shutdown]\n";
    sc.shutdown();
    return 0;
}
