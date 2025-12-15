#include "caddy_ai2_ros2_control_system_steering_driver/canopen_driver.hpp"
#include "caddy_ai2_ros2_control_system_steering_driver/socket_can_interface.hpp"
#include <cstring>
#include <iostream>

// Comandos NMT estándar
#define NMT_START_REMOTE_NODE    0x01
#define NMT_STOP_REMOTE_NODE     0x02
#define NMT_ENTER_PRE_OPERATIONAL 0x80
#define NMT_RESET_NODE           0x81
#define NMT_RESET_COMMUNICATION  0x82

CANOpenDriver::CANOpenDriver(uint8_t node_id, const std::string& name)
    : node_id_(node_id)
    , name_(name)
    , nmt_state_(NMTState::BOOT_UP)
    , can_interface_(nullptr)
    , communication_ok_(false) {
}

bool CANOpenDriver::sendNMT(uint8_t command) {
    if (!can_interface_) return false;

    struct can_frame frame;
    frame.can_id = 0x000;  // NMT COB-ID
    frame.can_dlc = 2;
    frame.data[0] = command;
    frame.data[1] = node_id_;

    return can_interface_->write(frame);
}

bool CANOpenDriver::sendSDO(uint16_t index, uint8_t subindex, const uint8_t* data, uint8_t data_len, bool write) {
    if (!can_interface_) return false;

    struct can_frame frame;
    frame.can_id = 0x600 + node_id_;  // SDO Request COB-ID
    frame.can_dlc = 8;

    if (write) {
        // SDO Download (write)
        frame.data[0] = 0x23;  // Command byte: download, 4 bytes
        frame.data[1] = index & 0xFF;
        frame.data[2] = (index >> 8) & 0xFF;
        frame.data[3] = subindex;
        memcpy(&frame.data[4], data, std::min(data_len, (uint8_t)4));
    } else {
        // SDO Upload (read)
        frame.data[0] = 0x40;  // Command byte: upload request
        frame.data[1] = index & 0xFF;
        frame.data[2] = (index >> 8) & 0xFF;
        frame.data[3] = subindex;
        memset(&frame.data[4], 0, 4);
    }

    return can_interface_->write(frame);
}

bool CANOpenDriver::sendPDO(uint16_t cob_id, const uint8_t* data, uint8_t data_len) {
    if (!can_interface_) return false;

    struct can_frame frame;
    frame.can_id = cob_id;
    frame.can_dlc = data_len;
    memcpy(frame.data, data, data_len);

    return can_interface_->write(frame);
}

bool CANOpenDriver::sendSync() {
    if (!can_interface_) return false;

    struct can_frame frame;
    frame.can_id = 0x80;  // SYNC COB-ID
    frame.can_dlc = 0;

    return can_interface_->write(frame);
}

bool CANOpenDriver::sendNodeGuard() {
    if (!can_interface_) return false;

    struct can_frame frame;
    frame.can_id = 0x700 + node_id_;  // Node Guard COB-ID
    frame.can_dlc = 1;
    frame.data[0] = 0x00;  // RTR flag se maneja diferente en SocketCAN

    return can_interface_->write(frame);
}

void CANOpenDriver::buildCANFrame(struct can_frame& frame, uint32_t can_id, const uint8_t* data, uint8_t len) {
    frame.can_id = can_id;
    frame.can_dlc = len;
    if (data && len > 0) {
        memcpy(frame.data, data, std::min(len, (uint8_t)8));
    }
}

bool CANOpenDriver::checkTimeout(std::chrono::steady_clock::time_point& last_time, int timeout_ms) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();
    return elapsed >= timeout_ms;
}

void CANOpenDriver::resetTimeout(std::chrono::steady_clock::time_point& time_point) {
    time_point = std::chrono::steady_clock::now();
}