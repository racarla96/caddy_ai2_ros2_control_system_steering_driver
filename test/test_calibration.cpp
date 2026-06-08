// Calibración del potenciómetro de dirección.
//
// Mueve la rueda lentamente entre los dos límites físicos mientras este test
// corre. Registra la correlación entre motor_pos (encoder del motor) y la
// entrada analógica leída por SDO 0x201A. Al salir con Ctrl+C imprime la
// regresión lineal y los parámetros URDF sugeridos.
//
// Uso:
//   ./test_calibration <can> [motor_id] [analog_pin]
//   analog_pin: 0-based (0=AI1, 1=AI2, 2=AI3), default=0

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include <linux/can.h>

#include "caddy_ai2_ros2_control_system_steering_driver/socket_can_interface.hpp"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// Posiciones del encoder del motor en cada límite físico (del URDF por defecto)
static constexpr int32_t ACTUATOR_LEFT  =  99100;
static constexpr int32_t ACTUATOR_RIGHT = -97500;
static constexpr int32_t ACTUATOR_ZERO  =    800;

static constexpr uint16_t SDO_IDX_AI = 0x201AU;

// ── Acumuladores para regresión lineal online ─────────────────────────────────
// y = a·x + b  donde  x = motor_pos,  y = AI_mV
struct Regression {
    int64_t n   {0};
    double  sx  {0}, sy  {0};
    double  sxx {0}, sxy {0}, syy {0};
    int32_t min_x{std::numeric_limits<int32_t>::max()};
    int32_t max_x{std::numeric_limits<int32_t>::min()};
    int32_t min_y{std::numeric_limits<int32_t>::max()};
    int32_t max_y{std::numeric_limits<int32_t>::min()};

    void add(int32_t x, int32_t y) {
        ++n;
        sx  += x;   sy  += y;
        sxx += static_cast<double>(x) * x;
        sxy += static_cast<double>(x) * y;
        syy += static_cast<double>(y) * y;
        if (x < min_x) min_x = x;
        if (x > max_x) max_x = x;
        if (y < min_y) min_y = y;
        if (y > max_y) max_y = y;
    }

    bool fit(double& a, double& b, double& r) const {
        if (n < 2) return false;
        const double den = n * sxx - sx * sx;
        if (std::abs(den) < 1e-9) return false;
        a = (n * sxy - sx * sy) / den;
        b = (sy - a * sx) / n;
        const double rn  = n * sxy - sx * sy;
        const double rd  = std::sqrt((n * sxx - sx * sx) * (n * syy - sy * sy));
        r = (rd > 1e-9) ? rn / rd : 0.0;
        return true;
    }
};

int main(int argc, char* argv[])
{
    const std::string can_iface = (argc > 1) ? argv[1] : "vcan0";
    const uint8_t motor_id   = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    const uint8_t analog_pin = (argc > 3) ? static_cast<uint8_t>(std::atoi(argv[3])) : 0;
    const uint8_t subindex   = analog_pin + 1;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Calibración Potenciómetro ===\n"
              << "  CAN       : " << can_iface << "\n"
              << "  motor_id  : " << static_cast<int>(motor_id) << "\n"
              << "  Analógica : AI" << static_cast<int>(subindex)
              << "  (pin idx " << static_cast<int>(analog_pin) << ")\n\n"
              << "  PASO 1: Asegúrate de que la rueda está RECTA (posición 0).\n"
              << "          El primer valor leído se usará como referencia de 0 rad.\n"
              << "  PASO 2: Mueve la rueda lentamente entre los dos límites físicos.\n"
              << "  PASO 3: Ctrl+C cuando hayas cubierto todo el rango.\n\n";

    SocketCANInterface can(can_iface);
    if (!can.init()) {
        std::cerr << "[ERROR] No se pudo abrir " << can_iface << "\n";
        return 1;
    }

    // NMT START broadcast
    {
        can_frame f{};
        f.can_id  = 0x000;
        f.can_dlc = 2;
        f.data[0] = 0x01;
        f.data[1] = 0x00;
        can.write(f);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << std::left
              << std::setw(12) << "motor_pos"
              << std::setw(10) << "AI_mV"
              << " |  "
              << std::setw(12) << "min_pos"
              << std::setw(12) << "max_pos"
              << std::setw(10) << "min_mV"
              << std::setw(10) << "max_mV"
              << "N\n"
              << std::string(78, '-') << "\n";

    Regression reg;
    int32_t cur_pos  = 0;
    int32_t cur_mv   = 0;
    bool    pos_ok   = false;
    bool    ai_ok    = false;
    int     cycle    = 0;
    int32_t zero_pos = 0;   // motor_pos en posición recta (primer dato válido)
    int32_t zero_mv  = 0;   // AI_mV     en posición recta (primer dato válido)
    bool    zero_set = false;

    while (g_running) {
        const auto t0 = std::chrono::steady_clock::now();

        // SYNC
        {
            can_frame f{};
            f.can_id  = 0x080;
            f.can_dlc = 0;
            can.write(f);
        }

        // SDO poll AI(subindex)
        {
            can_frame f{};
            f.can_id  = static_cast<canid_t>(0x600U + motor_id);
            f.can_dlc = 8;
            f.data[0] = 0x40;
            f.data[1] = static_cast<uint8_t>(SDO_IDX_AI & 0xFFU);
            f.data[2] = static_cast<uint8_t>((SDO_IDX_AI >> 8) & 0xFFU);
            f.data[3] = subindex;
            can.write(f);
        }

        // Drain
        std::vector<can_frame> frames;
        can.read(frames, nullptr, 5);

        for (const auto& fr : frames) {
            const uint32_t id = fr.can_id & ~static_cast<uint32_t>(CAN_RTR_FLAG);

            if (id == static_cast<uint32_t>(0x400U + motor_id) && fr.can_dlc >= 4) {
                std::memcpy(&cur_pos, fr.data, 4);
                pos_ok = true;
            } else if (id == static_cast<uint32_t>(0x580U + motor_id) && fr.can_dlc >= 6) {
                const uint8_t  cmd = fr.data[0];
                const uint16_t idx = static_cast<uint16_t>(fr.data[1]) |
                                     (static_cast<uint16_t>(fr.data[2]) << 8);
                const uint8_t  sub = fr.data[3];
                if ((cmd & 0xE0U) == 0x40U && idx == SDO_IDX_AI && sub == subindex) {
                    const int16_t raw = static_cast<int16_t>(
                        static_cast<uint16_t>(fr.data[4]) |
                        (static_cast<uint16_t>(fr.data[5]) << 8));
                    cur_mv = static_cast<int32_t>(raw) * 20000 / 16384;
                    ai_ok  = true;
                }
            }
        }

        if (pos_ok && ai_ok) {
            if (!zero_set) {
                zero_pos = cur_pos;
                zero_mv  = cur_mv;
                zero_set = true;
                std::cout << "[zero] Referencia 0 rad capturada:"
                          << "  motor_pos=" << zero_pos
                          << "  AI_mV=" << zero_mv << "\n\n";
            }
            reg.add(cur_pos, cur_mv);
        }

        ++cycle;

        // Actualizar línea cada 10 ciclos (200 ms)
        if (cycle % 10 == 0 && pos_ok && ai_ok) {
            std::cout << "\r" << std::left
                      << std::setw(12) << cur_pos
                      << std::setw(10) << cur_mv
                      << " |  "
                      << std::setw(12) << (reg.n > 0 ? reg.min_x : 0)
                      << std::setw(12) << (reg.n > 0 ? reg.max_x : 0)
                      << std::setw(10) << (reg.n > 0 ? reg.min_y : 0)
                      << std::setw(10) << (reg.n > 0 ? reg.max_y : 0)
                      << reg.n
                      << std::flush;
        }

        const auto elapsed   = std::chrono::steady_clock::now() - t0;
        const auto remaining = std::chrono::milliseconds(20) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    std::cout << "\n\n";

    if (reg.n < 10) {
        std::cout << "Pocas muestras (" << reg.n << "), datos insuficientes.\n";
        can.close();
        return 1;
    }

    double a = 0, b = 0, r = 0;
    const bool fit_ok = reg.fit(a, b, r);

    std::cout << std::string(60, '=') << "\n";
    std::cout << "  N muestras : " << reg.n << "\n\n";
    std::cout << "  motor_pos  min=" << reg.min_x << "  max=" << reg.max_x << "\n";
    std::cout << "  AI" << static_cast<int>(subindex)
              << "_mV    min=" << reg.min_y << "  max=" << reg.max_y << "\n";

    if (fit_ok) {
        std::cout << "\n  Regresión lineal:\n";
        std::cout << std::fixed << std::setprecision(5);
        std::cout << "    AI_mV = " << a << " × motor_pos + " << b << "\n";
        std::cout << std::setprecision(4);
        std::cout << "    Pearson r = " << r;
        if (std::abs(r) > 0.99)      std::cout << "  (excelente)";
        else if (std::abs(r) > 0.95) std::cout << "  (buena)";
        else                          std::cout << "  (débil — revisa el montaje)";
        std::cout << "\n\n";

        const int pot_left  = static_cast<int>(std::round(a * ACTUATOR_LEFT  + b));
        const int pot_right = static_cast<int>(std::round(a * ACTUATOR_RIGHT + b));
        // zero: valor medido directamente al inicio (más fiable que la predicción)
        const int pot_zero  = zero_set ? zero_mv
                                       : static_cast<int>(std::round(a * ACTUATOR_ZERO + b));

        std::cout << "  Parámetros URDF sugeridos\n";
        std::cout << "  (actuator_left=" << ACTUATOR_LEFT
                  << "  actuator_right=" << ACTUATOR_RIGHT << ")\n";
        if (zero_set)
            std::cout << "  (zero medido directamente: motor_pos=" << zero_pos
                      << "  AI_mV=" << zero_mv << ")\n";
        std::cout << "  " << std::string(56, '-') << "\n";
        std::cout << "  potentiometer_left_mv   = " << pot_left  << "\n";
        std::cout << "  potentiometer_right_mv  = " << pot_right << "\n";
        std::cout << "  potentiometer_zero_mv   = " << pot_zero
                  << (zero_set ? "  (medido)" : "  (predicción regresión)") << "\n";
    }

    std::cout << std::string(60, '=') << "\n";

    can.close();
    return 0;
}
