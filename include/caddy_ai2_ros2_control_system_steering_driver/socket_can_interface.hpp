#pragma once

#include <cstddef>
#include <string>
#include <vector>
#include <mutex>
#include <linux/can.h>
#include <sys/epoll.h>

class SocketCANInterface
{
public:
  enum class Status {
    Ok = 0,
    Timeout,
    NotInitialized,
    DeviceError,
    WouldBlock,
    InvalidFrame
  };

  explicit SocketCANInterface(const std::string& interface_name = "can0");
  ~SocketCANInterface();

  bool init();
  bool reset();
  void close();

  Status read(std::vector<struct can_frame>& frames, std::size_t* frames_read = nullptr, int timeout_ms = 100);
  Status write(const struct can_frame& frame);

  bool isInitialized() const;
  static const char* statusToString(Status status);

private:
  bool initUnlocked();
  void cleanupUnlocked();

  std::string interface_name_;
  int socket_fd_;
  int epoll_fd_;
  bool initialized_;
  mutable std::mutex mutex_;
};
