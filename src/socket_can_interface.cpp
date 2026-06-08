#include "caddy_ai2_ros2_control_system_steering_driver/socket_can_interface.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

SocketCANInterface::SocketCANInterface(const std::string& interface_name)
  : interface_name_(interface_name), socket_fd_(-1), epoll_fd_(-1), initialized_(false) {
}

SocketCANInterface::~SocketCANInterface() {
  close();
}

bool SocketCANInterface::init() {
  std::lock_guard<std::mutex> lock(mutex_);

  if (initialized_) {
    return true;
  }

  return initUnlocked();
}

bool SocketCANInterface::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  cleanupUnlocked();
  return initUnlocked();
}

bool SocketCANInterface::initUnlocked() {
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "Error al crear socket CAN: " << strerror(errno) << std::endl;
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "Error al obtener índice de interfaz: " << strerror(errno) << std::endl;
    cleanupUnlocked();
    return false;
  }

  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    std::cerr << "Error al vincular socket a interfaz: " << strerror(errno) << std::endl;
    cleanupUnlocked();
    return false;
  }

  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0) {
    std::cerr << "Error al obtener flags de socket: " << strerror(errno) << std::endl;
    cleanupUnlocked();
    return false;
  }

  flags |= O_NONBLOCK;
  if (fcntl(socket_fd_, F_SETFL, flags) < 0) {
    std::cerr << "Error al configurar socket como no bloqueante: " << strerror(errno) << std::endl;
    cleanupUnlocked();
    return false;
  }

  epoll_fd_ = epoll_create1(0);
  if (epoll_fd_ < 0) {
    std::cerr << "Error al crear epoll: " << strerror(errno) << std::endl;
    cleanupUnlocked();
    return false;
  }

  struct epoll_event ev;
  ev.events = EPOLLIN;
  ev.data.fd = socket_fd_;

  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, socket_fd_, &ev) < 0) {
    std::cerr << "Error al añadir socket a epoll: " << strerror(errno) << std::endl;
    cleanupUnlocked();
    return false;
  }

  initialized_ = true;
  std::cout << "Interfaz SocketCAN inicializada correctamente en " << interface_name_ << std::endl;
  return true;
}

void SocketCANInterface::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  cleanupUnlocked();
}

bool SocketCANInterface::isInitialized() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return initialized_;
}

const char* SocketCANInterface::statusToString(Status status) {
  switch (status) {
    case Status::Ok:
      return "Ok";
    case Status::Timeout:
      return "Timeout";
    case Status::NotInitialized:
      return "NotInitialized";
    case Status::DeviceError:
      return "DeviceError";
    case Status::WouldBlock:
      return "WouldBlock";
    case Status::InvalidFrame:
      return "InvalidFrame";
    default:
      return "Unknown";
  }
}

void SocketCANInterface::cleanupUnlocked() {
  if (epoll_fd_ >= 0) {
    ::close(epoll_fd_);
    epoll_fd_ = -1;
  }

  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }

  initialized_ = false;
}

SocketCANInterface::Status SocketCANInterface::read(std::vector<struct can_frame>& frames,
                                                    std::size_t* frames_read,
                                                    int timeout_ms) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (frames_read != nullptr) {
    *frames_read = 0;
  }

  if (!initialized_ || socket_fd_ < 0 || epoll_fd_ < 0) {
    std::cerr << "Interfaz no inicializada" << std::endl;
    return Status::NotInitialized;
  }

  frames.clear();

  struct epoll_event events[1];
  int ready = 0;

  while (true) {
    ready = epoll_wait(epoll_fd_, events, 1, timeout_ms);
    if (ready < 0 && errno == EINTR) {
      continue;
    }

    if (ready < 0) {
      std::cerr << "Error en epoll_wait: " << strerror(errno) << std::endl;
      return Status::DeviceError;
    }

    break;
  }

  if (ready == 0) {
    return Status::Timeout;
  }

  if (events[0].events & (EPOLLERR | EPOLLHUP)) {
    std::cerr << "Error o cierre detectado en la interfaz CAN" << std::endl;
    return Status::DeviceError;
  }

  if (!(events[0].events & EPOLLIN)) {
    return Status::Timeout;
  }

  std::size_t read_count = 0;
  bool saw_invalid_frame = false;

  while (true) {
    struct can_frame frame;
    ssize_t nbytes = ::read(socket_fd_, &frame, sizeof(struct can_frame));

    if (nbytes < 0) {
      if (errno == EINTR) {
        continue;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }

      std::cerr << "Error al leer mensaje CAN: " << strerror(errno) << std::endl;
      return Status::DeviceError;
    }

    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
      std::cerr << "Lectura incompleta de frame CAN" << std::endl;
      saw_invalid_frame = true;
      continue;
    }

    frames.push_back(frame);
    ++read_count;
  }

  if (frames_read != nullptr) {
    *frames_read = read_count;
  }

  if (saw_invalid_frame) {
    return Status::InvalidFrame;
  }

  return Status::Ok;
}

SocketCANInterface::Status SocketCANInterface::write(const struct can_frame& frame) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_ || socket_fd_ < 0) {
    std::cerr << "Interfaz no inicializada" << std::endl;
    return Status::NotInitialized;
  }

  while (true) {
    ssize_t nbytes = ::write(socket_fd_, &frame, sizeof(struct can_frame));

    if (nbytes == static_cast<ssize_t>(sizeof(struct can_frame))) {
      return Status::Ok;
    }

    if (nbytes < 0 && errno == EINTR) {
      continue;
    }

    if (nbytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      std::cerr << "Socket CAN temporalmente no disponible para escritura: " << strerror(errno) << std::endl;
      return Status::WouldBlock;
    }

    if (nbytes < 0) {
      std::cerr << "Error al enviar mensaje CAN: " << strerror(errno) << std::endl;
      return Status::DeviceError;
    }

    std::cerr << "Escritura incompleta de frame CAN" << std::endl;
    return Status::InvalidFrame;
  }
}
