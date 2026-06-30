// Calibración del encoder EPC (encoder absoluto externo).
//
// Configura el TPDO1 del encoder y lee en paralelo la posición del motor
// (TPDO21) y la del encoder (TPDO1). El usuario mueve la rueda lentamente
// entre los dos límites físicos; al salir con Ctrl+C se imprimen los
// parámetros URDF sugeridos.
//
// Uso:
//   ./test_calibration_epc <can> [motor_id] [encoder_id]
//   motor_id:   nodo CAN del drive DZCANTE (default=1)
//   encoder_id: nodo CAN del encoder EPC   (default=127)
//
// Procedimiento:
//   1. Pon la rueda RECTA (posición 0) antes de ejecutar.
//      El primer par (motor_pos, enc_pos) válido se registra como zero.
//   2. Mueve la rueda lentamente de un límite físico al otro.
//   3. Ctrl+C para salir y ver los parámetros URDF sugeridos.

#include <atomic>
#include <chrono>
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

static constexpr uint32_t COB_NMT          = 0x000U;
static constexpr uint32_t COB_SYNC         = 0x080U;
static constexpr uint32_t COB_SDO_REQ_BASE = 0x600U;
static constexpr uint32_t COB_TPDO21_BASE  = 0x400U;  // motor ActualPosition
static constexpr uint32_t COB_ENC_TPDO1_BASE = 0x180U;  // encoder position
static constexpr uint16_t SDO_IDX_TPDO1_COMM = 0x1800U;

static void send_nmt(SocketCANInterface& can, uint8_t cmd, uint8_t node_id)
{
    can_frame f{};
    f.can_id  = COB_NMT;
    f.can_dlc = 2;
    f.data[0] = cmd;
    f.data[1] = node_id;
    can.write(f);
}

static void send_sdo_write(SocketCANInterface& can, uint8_t node_id,
                            uint16_t idx, uint8_t sub, uint32_t val, uint8_t nbytes)
{
    can_frame f{};
    f.can_id  = static_cast<canid_t>(COB_SDO_REQ_BASE + node_id);
    f.can_dlc = 8;
    f.data[0] = (nbytes == 4) ? 0x23U : (nbytes == 2) ? 0x2BU : 0x2FU;
    f.data[1] = static_cast<uint8_t>(idx & 0xFFU);
    f.data[2] = static_cast<uint8_t>((idx >> 8) & 0xFFU);
    f.data[3] = sub;
    f.data[4] = static_cast<uint8_t>(val & 0xFFU);
    f.data[5] = static_cast<uint8_t>((val >> 8) & 0xFFU);
    f.data[6] = static_cast<uint8_t>((val >> 16) & 0xFFU);
    f.data[7] = static_cast<uint8_t>((val >> 24) & 0xFFU);
    can.write(f);
}

int main(int argc, char* argv[])
{
    const std::string can_iface  = (argc > 1) ? argv[1] : "vcan0";
    const uint8_t motor_id   = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    const uint8_t encoder_id = (argc > 3) ? static_cast<uint8_t>(std::atoi(argv[3])) : 127;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== Calibración Encoder EPC ===\n"
              << "  CAN        : " << can_iface << "\n"
              << "  motor_id   : " << static_cast<int>(motor_id) << "\n"
              << "  encoder_id : " << static_cast<int>(encoder_id) << "\n\n"
              << "  PASO 1: Asegúrate de que la rueda está RECTA (posición 0).\n"
              << "          El primer valor válido se registrará como zero.\n"
              << "  PASO 2: Mueve la rueda lentamente entre los dos límites físicos.\n"
              << "  PASO 3: Ctrl+C para ver los parámetros URDF sugeridos.\n\n";

    SocketCANInterface can(can_iface);
    if (!can.init()) {
        std::cerr << "[ERROR] No se pudo abrir " << can_iface << "\n";
        return 1;
    }

    // Configurar TPDO1 del encoder (COB-ID + tipo transmisión síncrona)
    std::cout << "[cfg] Configurando encoder TPDO1...\n";
    send_nmt(can, 0x82U, encoder_id);  // RESET_COMM solo al encoder
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    send_nmt(can, 0x80U, encoder_id);  // PRE_OP solo al encoder
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    send_sdo_write(can, encoder_id, SDO_IDX_TPDO1_COMM, 0x01,
                   COB_ENC_TPDO1_BASE + encoder_id, 4);  // COB-ID
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    send_sdo_write(can, encoder_id, SDO_IDX_TPDO1_COMM, 0x02, 1, 1);  // trans=SYNC
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // NMT START broadcast: motor y encoder pasan a OPERATIONAL y emiten TPDOs
    send_nmt(can, 0x01U, 0x00U);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << "[cfg] OK\n\n";

    std::cout << std::left
              << std::setw(12) << "motor_pos"
              << std::setw(12) << "enc_pos"
              << " |  "
              << std::setw(14) << "min_motor"
              << std::setw(14) << "max_motor"
              << std::setw(12) << "min_enc"
              << std::setw(12) << "max_enc"
              << "N\n"
              << std::string(84, '-') << "\n";

    int32_t cur_motor = 0;
    int32_t cur_enc   = 0;
    bool    motor_ok  = false;
    bool    enc_ok    = false;

    int32_t zero_motor = 0;
    int32_t zero_enc   = 0;
    bool    zero_set   = false;

    int32_t max_motor        = std::numeric_limits<int32_t>::min();
    int32_t min_motor        = std::numeric_limits<int32_t>::max();
    int32_t max_enc          = std::numeric_limits<int32_t>::min();
    int32_t min_enc          = std::numeric_limits<int32_t>::max();
    int32_t enc_at_max_motor = 0;
    int32_t enc_at_min_motor = 0;
    int64_t n_samples        = 0;

    int cycle = 0;

    while (g_running) {
        const auto t0 = std::chrono::steady_clock::now();

        // SYNC
        {
            can_frame f{};
            f.can_id  = COB_SYNC;
            f.can_dlc = 0;
            can.write(f);
        }

        std::vector<can_frame> frames;
        can.read(frames, nullptr, 5);

        for (const auto& fr : frames) {
            const uint32_t id = fr.can_id & ~static_cast<uint32_t>(CAN_RTR_FLAG);

            if (id == static_cast<uint32_t>(COB_TPDO21_BASE + motor_id) && fr.can_dlc >= 4) {
                std::memcpy(&cur_motor, fr.data, 4);
                motor_ok = true;
            } else if (id == static_cast<uint32_t>(COB_ENC_TPDO1_BASE + encoder_id)
                       && fr.can_dlc >= 4) {
                cur_enc = static_cast<int32_t>(
                    static_cast<uint32_t>(fr.data[0])          |
                    (static_cast<uint32_t>(fr.data[1]) <<  8)  |
                    (static_cast<uint32_t>(fr.data[2]) << 16)  |
                    (static_cast<uint32_t>(fr.data[3]) << 24));
                enc_ok = true;
            }
        }

        if (motor_ok && enc_ok) {
            if (!zero_set) {
                zero_motor = cur_motor;
                zero_enc   = cur_enc;
                zero_set   = true;
                std::cout << "[zero] Referencia 0 rad capturada:"
                          << "  motor_pos=" << zero_motor
                          << "  enc_pos=" << zero_enc << "\n\n";
            }

            if (cur_motor > max_motor) { max_motor = cur_motor; enc_at_max_motor = cur_enc; }
            if (cur_motor < min_motor) { min_motor = cur_motor; enc_at_min_motor = cur_enc; }
            if (cur_enc > max_enc)     max_enc = cur_enc;
            if (cur_enc < min_enc)     min_enc = cur_enc;
            ++n_samples;
        }

        ++cycle;

        if (cycle % 10 == 0 && motor_ok && enc_ok) {
            std::cout << "\r" << std::left
                      << std::setw(12) << cur_motor
                      << std::setw(12) << cur_enc
                      << " |  "
                      << std::setw(14) << (n_samples > 0 ? min_motor : 0)
                      << std::setw(14) << (n_samples > 0 ? max_motor : 0)
                      << std::setw(12) << (n_samples > 0 ? min_enc   : 0)
                      << std::setw(12) << (n_samples > 0 ? max_enc   : 0)
                      << n_samples
                      << std::flush;
        }

        const auto elapsed   = std::chrono::steady_clock::now() - t0;
        const auto remaining = std::chrono::milliseconds(20) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    std::cout << "\n\n";

    if (n_samples < 10) {
        std::cout << "Pocas muestras (" << n_samples << "), datos insuficientes.\n";
        can.close();
        return 1;
    }

    // Por convenio URDF: actuator_encoder_left > 0 (izquierda), < 0 (derecha)
    const bool left_is_max     = (std::abs(max_motor) >= std::abs(min_motor));
    const int32_t act_left     = left_is_max ? max_motor        : min_motor;
    const int32_t act_right    = left_is_max ? min_motor        : max_motor;
    const int32_t read_left    = left_is_max ? enc_at_max_motor : enc_at_min_motor;
    const int32_t read_right   = left_is_max ? enc_at_min_motor : enc_at_max_motor;

    std::cout << std::string(60, '=') << "\n";
    std::cout << "  N muestras : " << n_samples << "\n\n";
    std::cout << "  motor_pos  min=" << min_motor << "  max=" << max_motor << "\n";
    std::cout << "  enc_pos    min=" << min_enc   << "  max=" << max_enc   << "\n\n";
    std::cout << "  Parámetros URDF sugeridos\n";
    std::cout << "  " << std::string(56, '-') << "\n";
    std::cout << "  actuator_encoder_left         = " << act_left   << "\n";
    std::cout << "  actuator_encoder_right        = " << act_right  << "\n";
    std::cout << "  reading_encoder_left          = " << read_left  << "\n";
    std::cout << "  reading_encoder_right         = " << read_right << "\n";
    std::cout << "  reading_encoder_zero_position = " << zero_enc
              << "  (medido, motor_pos=" << zero_motor << ")\n";
    std::cout << std::string(60, '=') << "\n";

    can.close();
    return 0;
}
