// Test senoidal — seguimiento de referencia sinusoidal.
//
// Uso:
//   ./test_sinusoidal <can> <motor_id> <fb_mode> <fb_param> <min> <max> [freq_hz] [cycles]
//
//   fb_mode   0 = potenciómetro (entradas analógicas del DZCANTE)
//             1 = encoder EPC (nodo CANopen externo)
//   fb_param  Si fb_mode=0: pin analógico 0-based (0=AI1, 1=AI2, 2=AI3)
//             Si fb_mode=1: encoder CANopen node ID
//
// Ejemplos:
//   Potenciómetro en AI2:
//     ./test_sinusoidal can0 1 0 1 -97500 99100 0.05 3
//
//   Encoder EPC en nodo 127:
//     ./test_sinusoidal can0 1 1 127 -97500 99100 0.05 3

// robot@RBCAR-200101:~/ws_test2$ ./build/caddy_ai2_ros2_control_system_steering_driver/test_sinusoidal can_steer_drv 1 0 1 -90000 90000 0.05 3

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
    const uint8_t  motor_id     = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    const int      fb_mode      = (argc > 3) ? std::atoi(argv[3]) : 1;
    const int      fb_param     = (argc > 4) ? std::atoi(argv[4]) : (fb_mode == 0 ? 0 : 127);
    const int32_t  min_counts   = (argc > 5) ? static_cast<int32_t>(std::atol(argv[5])) : -10000;
    const int32_t  max_counts   = (argc > 6) ? static_cast<int32_t>(std::atol(argv[6])) :  10000;
    const double   freq_hz      = (argc > 7) ? std::atof(argv[7]) : 0.1;
    const double   num_cycles   = (argc > 8) ? std::atof(argv[8]) : 3.0;

    if (min_counts >= max_counts) {
        std::cerr << "[ERROR] min_counts debe ser < max_counts\n";
        return 1;
    }

    const bool    pot_mode   = (fb_mode == 0);
    const uint8_t encoder_id = pot_mode ? 0 : static_cast<uint8_t>(fb_param);
    const uint8_t analog_pin = pot_mode ? static_cast<uint8_t>(fb_param) : 0;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    const double center    = 0.5 * (min_counts + max_counts);
    const double amplitude = 0.5 * (max_counts - min_counts);
    const double period_s  = 1.0 / freq_hz;
    const double total_s   = num_cycles * period_s;

    std::cout << "=== Steering Driver — Test Senoidal ===\n"
              << "  CAN          : " << can_iface << "\n"
              << "  motor_id     : " << static_cast<int>(motor_id) << "\n"
              << "  Feedback     : ";
    if (pot_mode) {
        std::cout << "POTENCIÓMETRO  AI" << static_cast<int>(analog_pin + 1)
                  << " (pin idx " << static_cast<int>(analog_pin) << ")\n";
    } else {
        std::cout << "EPC ENCODER  node_id=" << static_cast<int>(encoder_id) << "\n";
    }
    std::cout << "  Rango motor  : [" << min_counts << ", " << max_counts << "] counts\n"
              << "  Sinusoide    : " << freq_hz << " Hz × " << num_cycles
              << " ciclos = " << total_s << " s\n"
              << "  Ctrl+C para salir\n\n";

    SteeringController sc(can_iface, motor_id, encoder_id, analog_pin);

    std::cout << "[init] Abriendo CAN y arrancando drive...\n";
    if (!sc.init()) {
        std::cerr << "[ERROR] init() falló.\n";
        return 1;
    }
    std::cout << "[init] OK\n\n";

    // ── Espera: solo DS402 OPERATION_ENABLED (no requiere feedback válido) ────
    constexpr double DT           = 0.02;
    constexpr double WAIT_TIMEOUT = 10.0;
    constexpr int    PRINT_EVERY  = 5;

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
                      << "  fault=" << (sc.has_fault() ? "SI" : "no") << "\n";
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

    // ── Seguimiento senoidal ──────────────────────────────────────────────────
    const char* fb_label = pot_mode ? "pot_mV" : "enc_pos";

    std::cout << std::left
              << std::setw(7)  << "t_s"
              << std::setw(10) << "ref"
              << std::setw(12) << "motor_pos"
              << std::setw(10) << "error"
              << std::setw(12) << fb_label
              << std::setw(14) << "NMT"
              << std::setw(10) << "DS402"
              << "\n" << std::string(75, '-') << "\n";

    constexpr int PRINT_EVERY_TEST = 10;  // cada 200 ms

    double test_elapsed = 0.0;
    int    test_iter    = 0;

    while (g_running && test_elapsed < total_s) {
        const auto t0 = std::chrono::steady_clock::now();

        const double ref_f = center + amplitude * std::sin(2.0 * M_PI * freq_hz * test_elapsed);
        const int32_t ref  = static_cast<int32_t>(std::round(ref_f));

        sc.set_target_position(ref);
        sc.step(DT);

        ++test_iter;
        test_elapsed += DT;

        if (test_iter % PRINT_EVERY_TEST == 0) {
            const int32_t motor_pos = sc.get_motor_position();
            const int32_t fb_val    = pot_mode
                                    ? static_cast<int32_t>(sc.get_analog_input_mv())
                                    : sc.get_position();
            const int32_t error     = ref - motor_pos;

            std::cout << std::left
                      << std::setw(7)  << std::fixed << std::setprecision(2) << test_elapsed
                      << std::setw(10) << ref
                      << std::setw(12) << motor_pos
                      << std::setw(10) << error
                      << std::setw(12) << fb_val
                      << std::setw(14) << nmt_to_str(sc.get_motor_nmt_state())
                      << std::setw(10) << drive_to_str(sc.get_motor_drive_state())
                      << "\n";
        }

        if (sc.has_fault()) {
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
