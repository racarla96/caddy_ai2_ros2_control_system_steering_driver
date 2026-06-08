#pragma once

#include <cstdint>

// ============================================================================
// Constantes del drive DZCANTE-020L080 de Advanced Motion Controls (AMC)
// Protocolo CANopen CiA 402, modo Profile Position
//
// IMPORTANTE: Los COB-IDs de PDO son propietarios de AMC y difieren del
// mapping estándar CANopen. NO usar los valores por defecto del estándar.
// ============================================================================

namespace dzcante020l080
{

// ----------------------------------------------------------------------------
// COB-IDs fijos (independientes del node_id)
// ----------------------------------------------------------------------------

// NMT request: frame de 2 bytes [comando, node_id] enviado por el master
constexpr uint32_t COB_NMT_REQUEST = 0x000U;

// SYNC: frame de 0 bytes que dispara los TPDOs síncronos del drive
constexpr uint32_t COB_SYNC = 0x080U;

// ----------------------------------------------------------------------------
// COB-IDs dependientes del node_id  (sumar node_id al base)
// ----------------------------------------------------------------------------

// SDO request del master al drive (expedited download / upload request)
constexpr uint32_t COB_SDO_REQUEST_BASE  = 0x600U;

// SDO response del drive al master
constexpr uint32_t COB_SDO_RESPONSE_BASE = 0x580U;

// NodeGuard RTR: master→drive con CAN_RTR_FLAG activo
// NMT heartbeat: drive→master en respuesta al RTR (mismo COB-ID, sin RTR)
constexpr uint32_t COB_NODEGUARD_BASE    = 0x700U;

// RPDO1: master→drive — ControlWord (2 bytes), transmisión ASYNC
// Configurado en el drive mediante SDO 0x1400:01
constexpr uint32_t COB_RPDO1_BASE        = 0x180U;

// RPDO21: master→drive — TargetPosition (4 bytes, int32 LE), transmisión ASYNC
// Configurado en el drive mediante SDO 0x1414:01
constexpr uint32_t COB_RPDO21_BASE       = 0x280U;

// TPDO1: drive→master — StatusWord (2 bytes), cada 10 SYNCs
// Configurado en el drive mediante SDO 0x1800:01
constexpr uint32_t COB_TPDO1_BASE        = 0x4A0U;

// TPDO21: drive→master — ActualPosition (4 bytes, int32 LE), cada SYNC
// Configurado en el drive mediante SDO 0x1814:01
constexpr uint32_t COB_TPDO21_BASE       = 0x400U;

// ----------------------------------------------------------------------------
// Comandos NMT (byte[0] del frame NMT request)
// ----------------------------------------------------------------------------

constexpr uint8_t NMT_START_REMOTE_NODE     = 0x01U;  // → OPERATIONAL
constexpr uint8_t NMT_STOP_REMOTE_NODE      = 0x02U;  // → STOPPED
constexpr uint8_t NMT_ENTER_PRE_OPERATIONAL = 0x80U;  // → PRE_OPERATIONAL
constexpr uint8_t NMT_RESET_NODE            = 0x81U;  // reset aplicación
constexpr uint8_t NMT_RESET_COMMUNICATION   = 0x82U;  // reset comunicación

// ----------------------------------------------------------------------------
// Bytes de estado en el heartbeat / NodeGuard response (byte[0] del frame)
// Los valores con bit 7 activo (0x8X) indican toggle-bit a 1 (normal tras boot)
// ----------------------------------------------------------------------------

constexpr uint8_t NMT_HB_OPERATIONAL_0     = 0x05U;
constexpr uint8_t NMT_HB_OPERATIONAL_1     = 0x85U;
constexpr uint8_t NMT_HB_STOPPED_0         = 0x04U;
constexpr uint8_t NMT_HB_STOPPED_1         = 0x84U;
constexpr uint8_t NMT_HB_PRE_OPERATIONAL_0 = 0x7FU;
constexpr uint8_t NMT_HB_PRE_OPERATIONAL_1 = 0xFFU;

// ----------------------------------------------------------------------------
// ControlWord DS402 (objeto SDO 0x6040:00 o RPDO1)
// ----------------------------------------------------------------------------

constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000U;
constexpr uint16_t CW_SHUTDOWN        = 0x0006U;
constexpr uint16_t CW_SWITCH_ON       = 0x0007U;
constexpr uint16_t CW_ENABLE_OP       = 0x000FU;
constexpr uint16_t CW_QUICK_STOP      = 0x0002U;
constexpr uint16_t CW_FAULT_RESET     = 0x0080U;

// ----------------------------------------------------------------------------
// Máscaras y valores del StatusWord DS402 (objeto TPDO1)
// Extraídas de updateMotorState() del driver original
// ----------------------------------------------------------------------------

constexpr uint16_t SW_MASK_SOD   = 0x4FU;  // máscara para SWITCH_ON_DISABLED y FAULT
constexpr uint16_t SW_MASK_STATE = 0x6FU;  // máscara para los demás estados

constexpr uint16_t SW_SWITCH_ON_DISABLED  = 0x40U;  // (sw & 0x4F) == 0x40
constexpr uint16_t SW_READY_TO_SWITCH_ON  = 0x21U;  // (sw & 0x6F) == 0x21
constexpr uint16_t SW_OPERATION_DISABLED  = 0x23U;  // (sw & 0x6F) == 0x23
constexpr uint16_t SW_OPERATION_ENABLED   = 0x27U;  // (sw & 0x6F) == 0x27
constexpr uint16_t SW_QUICK_STOP_ACTIVE   = 0x07U;  // (sw & 0x6F) == 0x07
constexpr uint16_t SW_FAULT               = 0x08U;  // (sw & 0x4F) == 0x08

// ----------------------------------------------------------------------------
// Índices SDO para configuración de PDOs y parámetros del drive
// ----------------------------------------------------------------------------

// RPDO1 communication parameters (ControlWord)
constexpr uint16_t SDO_IDX_RPDO1_COMM  = 0x1400U;
// RPDO21 communication parameters (TargetPosition)
constexpr uint16_t SDO_IDX_RPDO21_COMM = 0x1414U;
// TPDO1 communication parameters (StatusWord)
constexpr uint16_t SDO_IDX_TPDO1_COMM  = 0x1800U;
// TPDO21 communication parameters (ActualPosition)
constexpr uint16_t SDO_IDX_TPDO21_COMM = 0x1814U;

// Subíndices comunes en los comm parameters de PDO
constexpr uint8_t SDO_SUB_PDO_COB_ID           = 0x01U;  // COB-ID del PDO
constexpr uint8_t SDO_SUB_PDO_TRANSMISSION_TYPE = 0x02U;  // tipo de transmisión

// NodeGuard: guard time (ms, uint16)
constexpr uint16_t SDO_IDX_GUARD_TIME   = 0x100CU;
constexpr uint8_t  SDO_SUB_GUARD_TIME   = 0x00U;

// NodeGuard: life time factor (uint8)
constexpr uint16_t SDO_IDX_LIFE_FACTOR  = 0x100DU;
constexpr uint8_t  SDO_SUB_LIFE_FACTOR  = 0x00U;

// Modes of operation: 0x01 = Profile Position Mode
constexpr uint16_t SDO_IDX_MODES_OF_OP  = 0x6060U;
constexpr uint8_t  SDO_SUB_MODES_OF_OP  = 0x00U;
constexpr uint8_t  DRIVE_MODE_POSITION   = 0x01U;

// ControlWord (usado por SDO en PRE_OPERATIONAL durante startup)
constexpr uint16_t SDO_IDX_CONTROL_WORD = 0x6040U;
constexpr uint8_t  SDO_SUB_CONTROL_WORD = 0x00U;

// ----------------------------------------------------------------------------
// Tipos de transmisión de PDO
// ----------------------------------------------------------------------------

// TPDO1: StatusWord cada 10 SYNCs
constexpr uint8_t TPDO1_TRANSMISSION_TYPE  = 10U;
// TPDO21: ActualPosition cada SYNC
constexpr uint8_t TPDO21_TRANSMISSION_TYPE = 1U;
// RPDO ASYNC: el drive procesa el PDO inmediatamente al recibirlo
constexpr uint8_t RPDO_TRANSMISSION_TYPE_ASYNC = 0xFFU;

// Bit 31 del COB-ID de PDO comm parameter: 1 = PDO deshabilitado
constexpr uint32_t PDO_COB_ID_DISABLE_BIT = 0x80000000U;

// ----------------------------------------------------------------------------
// Bytes de comando SDO (expedited)
// ----------------------------------------------------------------------------

// SDO Download (write al drive) según número de bytes de datos
constexpr uint8_t SDO_CMD_WRITE_4B = 0x23U;  // 4 bytes de datos
constexpr uint8_t SDO_CMD_WRITE_2B = 0x2BU;  // 2 bytes de datos
constexpr uint8_t SDO_CMD_WRITE_1B = 0x2FU;  // 1 byte  de datos
// SDO Upload request (read del drive)
constexpr uint8_t SDO_CMD_READ_REQ  = 0x40U;

// ----------------------------------------------------------------------------
// Valores de configuración de NodeGuard
// ----------------------------------------------------------------------------

// Guard time en milisegundos (0x100C). El drive envía heartbeat cada guard_time ms.
constexpr uint16_t NODEGUARD_GUARD_TIME_MS = 200U;
// Life time factor (0x100D). Timeout = guard_time * life_factor ms.
constexpr uint8_t  NODEGUARD_LIFE_FACTOR   = 15U;

// ----------------------------------------------------------------------------
// Timeouts y períodos del ciclo de control (en segundos)
// ----------------------------------------------------------------------------

// Tiempo sin heartbeat del drive antes de declarar NMT FAULT
constexpr double NMT_TIMEOUT_S       = 2.0;
// Período de envío del NodeGuard RTR
constexpr double NODEGUARD_PERIOD_S  = 0.2;
// Intervalo mínimo entre transiciones DS402 (evita saturar el drive)
constexpr double DS402_MIN_INTERVAL_S = 0.05;

// Duración de los sleeps en la secuencia de arranque (milisegundos)
constexpr int STARTUP_SLEEP_RESET_MS  = 100;
constexpr int STARTUP_SLEEP_PREOP_MS  = 50;
constexpr int STARTUP_SLEEP_SDO_MS    = 50;
constexpr int STARTUP_SLEEP_PDO_MS    = 10;

// ----------------------------------------------------------------------------
// COB-IDs del encoder absoluto CAN externo
// El encoder usa el mapping ESTÁNDAR de CANopen (diferente del motor AMC)
// ----------------------------------------------------------------------------

// TPDO1 del encoder: posición absoluta (4 bytes LE), triggereado por SYNC
// COB-ID = 0x180 + enc_node_id  (standard CANopen TPDO1, mismo base que el
// RPDO1 del motor pero con distinto node_id → no hay colisión en el bus)
constexpr uint32_t COB_ENC_TPDO1_BASE = 0x180U;

// Tipo de transmisión del TPDO1 del encoder: cada SYNC (mismo que TPDO21 del motor)
constexpr uint8_t ENC_TPDO1_TRANSMISSION_TYPE = 1U;

// ----------------------------------------------------------------------------
// Entradas analógicas del DZCANTE (variante potenciómetro)
// Leídas por SDO upload a índice propietario AMC 0x201A.
// Subindex 1=AI1, 2=AI2, 3=AI3.
// Respuesta: int16 signed, escala ±16384 → ±20 V  (raw × 20000 / 16384 → mV)
// ----------------------------------------------------------------------------

constexpr uint16_t SDO_IDX_ANALOG_INPUT = 0x201AU;
// Escala: voltios = raw × 20.0 / 16384  →  mV = raw × 20000 / 16384
constexpr int32_t  ANALOG_INPUT_MV_NUM  = 20000;   // numerador
constexpr int32_t  ANALOG_INPUT_MV_DEN  = 16384;   // denominador

}  // namespace dzcante020l080
