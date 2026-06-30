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

#include "caddy_ai2_ros2_common/socket_can_interface.hpp"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static constexpr uint32_t COB_NMT            = 0x000U;
static constexpr uint32_t COB_SYNC           = 0x080U;
static constexpr uint32_t COB_SDO_REQ_BASE   = 0x600U;
static constexpr uint32_t COB_TPDO21_BASE    = 0x400U;   // motor ActualPosition
static constexpr uint32_t COB_ENC_TPDO1_BASE = 0x180U;   // encoder position
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

// Drena todos los frames disponibles. Llama varias veces para capturar frames
// que llegan con distinto retardo tras el SYNC.
static int drain_frames(SocketCANInterface& can, std::vector<can_frame>& out,
                        int total_ms, int slice_ms = 3)
{
    int total = 0;
    const auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(total_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        std::vector<can_frame> batch;
        int n = can.read(batch, slice_ms);
        if (n > 0) {
            out.insert(out.end(), batch.begin(), batch.end());
            total += n;
        }
    }
    return total;
}

static void print_frame_raw(const can_frame& fr)
{
    const uint32_t id = fr.can_id & ~static_cast<uint32_t>(CAN_RTR_FLAG);
    std::cout << "    0x" << std::hex << std::setw(3) << std::setfill('0') << id
              << std::dec << std::setfill(' ')
              << "  dlc=" << static_cast<int>(fr.can_dlc) << "  [";
    for (int i = 0; i < 8; ++i) {
        if (i < fr.can_dlc)
            std::cout << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(fr.data[i]);
        else
            std::cout << "--";
        if (i < 7) std::cout << " ";
    }
    std::cout << std::dec << std::setfill(' ') << "]\n";
}

int main(int argc, char* argv[])
{
    const std::string can_iface  = (argc > 1) ? argv[1] : "vcan0";
    const uint8_t motor_id   = (argc > 2) ? static_cast<uint8_t>(std::atoi(argv[2])) : 1;
    const uint8_t encoder_id = (argc > 3) ? static_cast<uint8_t>(std::atoi(argv[3])) : 127;

    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    const uint32_t expected_motor_cob = COB_TPDO21_BASE    + motor_id;
    const uint32_t expected_enc_cob   = COB_ENC_TPDO1_BASE + encoder_id;

    std::cout << "=== Calibración Encoder EPC ===\n"
              << "  CAN        : " << can_iface << "\n"
              << "  motor_id   : " << static_cast<int>(motor_id)
              << "  (TPDO21 COB-ID=0x" << std::hex << expected_motor_cob << std::dec << ")\n"
              << "  encoder_id : " << static_cast<int>(encoder_id)
              << "  (TPDO1  COB-ID=0x" << std::hex << expected_enc_cob  << std::dec << ")\n\n"
              << "  PASO 1: Asegúrate de que la rueda está RECTA (posición 0).\n"
              << "          El primer valor válido se registrará como zero.\n"
              << "  PASO 2: Mueve la rueda lentamente entre los dos límites físicos.\n"
              << "  PASO 3: Ctrl+C para ver los parámetros URDF sugeridos.\n\n";

    SocketCANInterface can(can_iface);
    if (!can.init()) {
        std::cerr << "[ERROR] No se pudo abrir " << can_iface << "\n";
        return 1;
    }

    // ── Configurar TPDO1 del encoder ────────────────────────────────────────────
    std::cout << "[cfg] Configurando encoder TPDO1...\n";
    send_nmt(can, 0x82U, encoder_id);   // RESET_COMM al encoder
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    send_nmt(can, 0x80U, encoder_id);   // PRE_OP al encoder
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Primero deshabilitar el TPDO (bit31=1) antes de cambiar COB-ID (CiA301 §7.2.4)
    send_sdo_write(can, encoder_id, SDO_IDX_TPDO1_COMM, 0x01,
                   (COB_ENC_TPDO1_BASE + encoder_id) | 0x80000000UL, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // Poner nuevo COB-ID habilitado (bit31=0)
    send_sdo_write(can, encoder_id, SDO_IDX_TPDO1_COMM, 0x01,
                   COB_ENC_TPDO1_BASE + encoder_id, 4);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // Tipo de transmisión = 1 (sincrónico: enviar tras cada SYNC)
    send_sdo_write(can, encoder_id, SDO_IDX_TPDO1_COMM, 0x02, 1U, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    // NMT START broadcast
    send_nmt(can, 0x01U, 0x00U);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    std::cout << "[cfg] OK\n\n";

    // ── Diagnóstico: escuchar el bus durante 2 s sin SYNC ───────────────────────
    std::cout << "[diag] Escuchando bus 2 s (sin SYNC) — frames recibidos:\n";
    {
        std::vector<can_frame> diag;
        drain_frames(can, diag, 2000, 10);
        if (diag.empty()) {
            std::cout << "  (ninguno)\n";
        } else {
            for (const auto& fr : diag) {
                const uint32_t id = fr.can_id & ~static_cast<uint32_t>(CAN_RTR_FLAG);
                std::string label;
                if (id == expected_motor_cob)       label = " ← TPDO21 motor";
                else if (id == expected_enc_cob)    label = " ← TPDO1  encoder";
                else if (id == (0x700U + motor_id)) label = " ← NMT motor";
                else if (id == (0x700U + encoder_id)) label = " ← NMT encoder";
                print_frame_raw(fr);
                if (!label.empty()) std::cout << "          " << label << "\n";
            }
        }
    }
    std::cout << "\n";

    // Si el encoder envía en modo asíncrono (sin SYNC), avisamos
    // pero continuamos con SYNC para el logging correlacionado.

    // ── Diagnóstico: un ciclo de SYNC y ver qué responde ────────────────────────
    std::cout << "[diag] Enviando 5 SYNCs de prueba:\n";
    for (int i = 0; i < 5 && g_running; ++i) {
        can_frame sync{};
        sync.can_id  = COB_SYNC;
        sync.can_dlc = 0;
        can.write(sync);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));  // dar tiempo al encoder

        std::vector<can_frame> batch;
        drain_frames(can, batch, 15, 3);  // 15 ms de ventana
        for (const auto& fr : batch) {
            const uint32_t id = fr.can_id & ~static_cast<uint32_t>(CAN_RTR_FLAG);
            std::string label;
            if (id == expected_motor_cob)       label = " ← TPDO21 motor";
            else if (id == expected_enc_cob)    label = " ← TPDO1  encoder";
            else if (id == (0x700U + motor_id)) label = " ← NMT motor";
            else if (id == (0x700U + encoder_id)) label = " ← NMT encoder";
            print_frame_raw(fr);
            if (!label.empty()) std::cout << "          " << label << "\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    std::cout << "\n";

    // ── Calibración ─────────────────────────────────────────────────────────────
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

    int cycle         = 0;
    int unknown_count = 0;

    while (g_running) {
        const auto t0 = std::chrono::steady_clock::now();

        // SYNC
        {
            can_frame f{};
            f.can_id  = COB_SYNC;
            f.can_dlc = 0;
            can.write(f);
        }

        // Esperar 10 ms para que TODOS los dispositivos respondan al SYNC
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Drenar todo lo que haya en el buffer
        std::vector<can_frame> frames;
        drain_frames(can, frames, 5, 2);

        for (const auto& fr : frames) {
            const uint32_t id = fr.can_id & ~static_cast<uint32_t>(CAN_RTR_FLAG);

            if (id == expected_motor_cob && fr.can_dlc >= 4) {
                std::memcpy(&cur_motor, fr.data, 4);
                motor_ok = true;
            } else if (id == expected_enc_cob && fr.can_dlc >= 4) {
                cur_enc = static_cast<int32_t>(
                    static_cast<uint32_t>(fr.data[0])          |
                    (static_cast<uint32_t>(fr.data[1]) <<  8)  |
                    (static_cast<uint32_t>(fr.data[2]) << 16)  |
                    (static_cast<uint32_t>(fr.data[3]) << 24));
                enc_ok = true;
            } else if (id != COB_SYNC && id != (0x700U + motor_id)
                       && id != (0x700U + encoder_id)
                       && id != (0x580U + motor_id)
                       && id != (0x580U + encoder_id)) {
                ++unknown_count;
            }
        }

        if (motor_ok && enc_ok) {
            if (!zero_set) {
                zero_motor = cur_motor;
                zero_enc   = cur_enc;
                zero_set   = true;
                std::cout << "\n[zero] Referencia 0 rad capturada:"
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

        // Resumen cada 10 ciclos (~200 ms)
        if (cycle % 10 == 0) {
            std::cout << "\r" << std::left
                      << std::setw(12) << (motor_ok ? std::to_string(cur_motor) : "?")
                      << std::setw(12) << (enc_ok   ? std::to_string(cur_enc)   : "?")
                      << " |  "
                      << std::setw(14) << (n_samples > 0 ? std::to_string(min_motor) : "-")
                      << std::setw(14) << (n_samples > 0 ? std::to_string(max_motor) : "-")
                      << std::setw(12) << (n_samples > 0 ? std::to_string(min_enc)   : "-")
                      << std::setw(12) << (n_samples > 0 ? std::to_string(max_enc)   : "-")
                      << n_samples
                      << std::flush;
        }

        // Completar ciclo de 20 ms (ya dormimos 10ms + drain ~5ms → ~5ms restantes)
        const auto elapsed   = std::chrono::steady_clock::now() - t0;
        const auto remaining = std::chrono::milliseconds(20) - elapsed;
        if (remaining > std::chrono::milliseconds(0))
            std::this_thread::sleep_for(remaining);
    }

    std::cout << "\n\n";

    if (n_samples < 10) {
        std::cout << "Pocas muestras (" << n_samples << "), datos insuficientes.\n";
        if (!motor_ok)
            std::cout << "  → MOTOR: ningún frame TPDO21 recibido en COB-ID 0x"
                      << std::hex << expected_motor_cob << std::dec << "\n";
        if (!enc_ok)
            std::cout << "  → ENCODER: ningún frame TPDO1 recibido en COB-ID 0x"
                      << std::hex << expected_enc_cob << std::dec << "\n";
        can.close();
        return 1;
    }

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
