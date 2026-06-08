// Test pasivo — escucha el bus CAN sin habilitar el motor.
//
// Uso:
//   ./test_passive <can> [motor_id] [analog_pin]
//
// Envía NMT START (broadcast) para que el drive empiece a mandar TPDOs,
// luego envía SYNC a 50 Hz y muestra todos los frames recibidos.
// Útil para verificar qué COB-IDs transmite el drive y leer entradas analógicas
// sin arrancar la secuencia DS402.
//
// Imprime:
//   - Una línea por frame con COB-ID, DLC, bytes en hex y decodificado
//   - Una línea de resumen cada segundo con AI1/AI2/AI3, posición y estado

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>

#include <linux/can.h>

#include "caddy_ai2_ros2_control_system_steering_driver/socket_can_interface.hpp"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// SDO object 0x201A: entradas analógicas propietarias del DZCANTE
static constexpr uint16_t SDO_IDX_AI = 0x201AU;

// ── Estado decodificado acumulado ─────────────────────────────────────────────
struct State {
    int16_t  ai_sdo[3]{};    // mV leídos por SDO 0x201A
    bool     ai_valid{false};
    int32_t  motor_pos{0};
    bool     pos_valid{false};
    uint16_t status_word{0};
    bool     sw_valid{false};
    uint8_t  nmt_byte{0xFF};
};

static void decode_frame(const can_frame& frame, uint8_t motor_id,
                         uint8_t analog_pin, State& st,
                         bool print_raw)
{
    const uint32_t bare_id = frame.can_id & ~static_cast<uint32_t>(CAN_RTR_FLAG);

    if (print_raw) {
        std::cout << "  0x" << std::hex << std::setw(3) << std::setfill('0') << bare_id
                  << std::dec << std::setfill(' ')
                  << "  dlc=" << static_cast<int>(frame.can_dlc)
                  << "  [";
        for (int i = 0; i < 8; ++i) {
            if (i < frame.can_dlc)
                std::cout << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(frame.data[i]);
            else
                std::cout << "--";
            if (i < 7) std::cout << " ";
        }
        std::cout << std::dec << std::setfill(' ') << "]  ";
    }

    // Decode por COB-ID
    if (bare_id == static_cast<uint32_t>(0x4A0U + motor_id)) {
        // TPDO1: StatusWord
        if (frame.can_dlc >= 2) {
            st.status_word = static_cast<uint16_t>(frame.data[0]) |
                             (static_cast<uint16_t>(frame.data[1]) << 8);
            st.sw_valid = true;
            if (print_raw)
                std::cout << "TPDO1-SW  sw=0x" << std::hex << st.status_word << std::dec;
        }
    } else if (bare_id == static_cast<uint32_t>(0x400U + motor_id)) {
        // TPDO21: ActualPosition
        if (frame.can_dlc >= 4) {
            std::memcpy(&st.motor_pos, frame.data, 4);
            st.pos_valid = true;
            if (print_raw)
                std::cout << "TPDO21-POS  pos=" << st.motor_pos;
        }
    } else if (bare_id == static_cast<uint32_t>(0x700U + motor_id)) {
        // NodeGuard / Heartbeat
        if (!(frame.can_id & CAN_RTR_FLAG) && frame.can_dlc >= 1) {
            st.nmt_byte = frame.data[0] & 0x7F;
            if (print_raw) {
                const char* s = (st.nmt_byte == 0x04) ? "OPERATIONAL"
                              : (st.nmt_byte == 0x7F) ? "PRE_OP"
                              : (st.nmt_byte == 0x05) ? "STOPPED"
                              : "?";
                std::cout << "NMT  state=" << s
                          << "(0x" << std::hex << static_cast<int>(st.nmt_byte) << std::dec << ")";
            }
        }
    } else if (bare_id == static_cast<uint32_t>(0x580U + motor_id)) {
        // SDO response — decodificar 0x201A (entradas analógicas)
        if (frame.can_dlc >= 6) {
            const uint8_t  cmd = frame.data[0];
            const uint16_t idx = static_cast<uint16_t>(frame.data[1]) |
                                 (static_cast<uint16_t>(frame.data[2]) << 8);
            const uint8_t  sub = frame.data[3];
            if ((cmd & 0xE0U) == 0x40U && idx == SDO_IDX_AI && sub >= 1 && sub <= 3) {
                const int16_t raw = static_cast<int16_t>(
                    static_cast<uint16_t>(frame.data[4]) |
                    (static_cast<uint16_t>(frame.data[5]) << 8));
                const int16_t mv = static_cast<int16_t>(
                    static_cast<int32_t>(raw) * 20000 / 16384);
                st.ai_sdo[sub - 1] = mv;
                st.ai_valid = true;
                if (print_raw) {
                    std::cout << "SDO-AI  AI" << static_cast<int>(sub)
                              << "=" << mv << "mV  (raw=" << raw << ")";
                }
            } else if (print_raw) {
                std::cout << "SDO-RSP  cmd=0x" << std::hex
                          << static_cast<int>(cmd) << std::dec
                          << "  idx=0x" << std::hex << idx << std::dec
                          << "  sub=" << static_cast<int>(sub);
            }
        }
    }

    if (print_raw) std::cout << "\n";
}

int main(int argc, char* argv[])
{
    const std::string can_iface = (argc > 1) ? argv[1] : "vcan0";
    const uint8_t motor_id      = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    const uint8_t analog_pin    = (argc > 3) ? static_cast<uint8_t>(std::atoi(argv[3])) : 0;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Steering Driver — Test Pasivo ===\n"
              << "  CAN         : " << can_iface << "\n"
              << "  motor_id    : " << static_cast<int>(motor_id) << "\n"
              << "  analog_pin  : " << static_cast<int>(analog_pin)
              << "  (AI" << static_cast<int>(analog_pin + 1) << ")\n"
              << "\n"
              << "  Envía NMT START para que el drive emita TPDOs,\n"
              << "  luego SYNC a 50 Hz. SIN secuencia DS402.\n"
              << "  Ctrl+C para salir\n\n";

    SocketCANInterface can(can_iface);
    if (!can.init()) {
        std::cerr << "[ERROR] No se pudo abrir la interfaz " << can_iface << "\n";
        return 1;
    }

    // NMT START broadcast → todos los nodos pasan a OPERATIONAL y emiten TPDOs
    {
        can_frame f{};
        f.can_id  = 0x000;
        f.can_dlc = 2;
        f.data[0] = 0x01;  // START_REMOTE_NODE
        f.data[1] = 0x00;  // todos los nodos
        can.write(f);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << "[Frames recibidos]\n";
    std::cout << std::string(72, '-') << "\n";

    State st{};
    int cycle      = 0;
    int print_rate = 50;  // resumen cada segundo (50 ciclos × 20 ms)

    while (g_running) {
        const auto t0 = std::chrono::steady_clock::now();

        // SYNC
        {
            can_frame f{};
            f.can_id  = 0x080;
            f.can_dlc = 0;
            can.write(f);
        }

        // SDO poll: pedir lectura analógica AI(analog_pin+1) por objeto 0x201A
        {
            can_frame f{};
            f.can_id  = static_cast<canid_t>(0x600U + motor_id);
            f.can_dlc = 8;
            f.data[0] = 0x40;  // upload request
            f.data[1] = static_cast<uint8_t>(SDO_IDX_AI & 0xFFU);
            f.data[2] = static_cast<uint8_t>((SDO_IDX_AI >> 8) & 0xFFU);
            f.data[3] = static_cast<uint8_t>(analog_pin + 1);
            can.write(f);
        }

        // Drain: 5 ms de espera para recoger la respuesta al SYNC
        std::vector<can_frame> frames;
        can.read(frames, nullptr, 5);

        const bool print_raw = true;  // imprime cada frame
        for (const auto& frame : frames)
            decode_frame(frame, motor_id, analog_pin, st, print_raw);

        ++cycle;

        // Resumen compacto cada segundo
        if (cycle % print_rate == 0) {
            std::cout << std::string(72, '-') << "\n";
            std::cout << "[Resumen t=" << std::fixed << std::setprecision(1)
                      << (cycle * 0.02) << "s]\n";
            if (st.ai_valid) {
                std::cout << "  AI1=" << st.ai_sdo[0] << "mV"
                          << "  AI2=" << st.ai_sdo[1] << "mV"
                          << "  AI3=" << st.ai_sdo[2] << "mV"
                          << "  --> pin" << static_cast<int>(analog_pin)
                          << "=" << st.ai_sdo[analog_pin] << "mV\n";
            } else {
                std::cout << "  SDO 0x201A: sin respuesta aún...\n";
            }
            if (st.pos_valid)
                std::cout << "  motor_pos=" << st.motor_pos << "\n";
            if (st.sw_valid)
                std::cout << "  status_word=0x" << std::hex << st.status_word << std::dec << "\n";
            std::cout << std::string(72, '-') << "\n";
        }

        const auto elapsed   = std::chrono::steady_clock::now() - t0;
        const auto remaining = std::chrono::milliseconds(20) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    can.close();
    std::cout << "\n[done]\n";
    return 0;
}
