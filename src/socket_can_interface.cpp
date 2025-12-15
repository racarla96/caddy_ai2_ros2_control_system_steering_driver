#include "caddy_ai2_ros2_control_system_steering_driver/socket_can_interface.hpp"

#include <cstring>
#include <iostream>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <fcntl.h>
#include <errno.h>

// Incluir libsocketcan si es necesario
// #include <libsocketcan.h>

SocketCANInterface::SocketCANInterface(const std::string& interface_name)
  : interface_name_(interface_name), socket_fd_(-1), epoll_fd_(-1), initialized_(false) {
}

SocketCANInterface::~SocketCANInterface() {
  close();
}

bool SocketCANInterface::init() {
  // Crear socket CAN en modo RAW
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    std::cerr << "Error al crear socket CAN: " << strerror(errno) << std::endl;
    return false;
  }

  // Configurar interfaz CAN
  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    std::cerr << "Error al obtener índice de interfaz: " << strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Vincular socket a la interfaz CAN
  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    std::cerr << "Error al vincular socket a interfaz: " << strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Configurar socket como no bloqueante
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0) {
    std::cerr << "Error al obtener flags de socket: " << strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }
  
  flags |= O_NONBLOCK;
  if (fcntl(socket_fd_, F_SETFL, flags) < 0) {
    std::cerr << "Error al configurar socket como no bloqueante: " << strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  // Configurar epoll para lectura eficiente
  epoll_fd_ = epoll_create1(0);
  if (epoll_fd_ < 0) {
    std::cerr << "Error al crear epoll: " << strerror(errno) << std::endl;
    ::close(socket_fd_);
    socket_fd_ = -1;
    return false;
  }

  struct epoll_event ev;
  ev.events = EPOLLIN;
  ev.data.fd = socket_fd_;
  
  if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, socket_fd_, &ev) < 0) {
    std::cerr << "Error al añadir socket a epoll: " << strerror(errno) << std::endl;
    ::close(epoll_fd_);
    ::close(socket_fd_);
    epoll_fd_ = -1;
    socket_fd_ = -1;
    return false;
  }

  initialized_ = true;
  std::cout << "Interfaz SocketCAN inicializada correctamente en " << interface_name_ << std::endl;
  return true;
}

void SocketCANInterface::close() {
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

// Leer mensajes CAN con tiempo de espera configurable
int SocketCANInterface::read(std::vector<struct can_frame>& frames, int timeout_ms) {
  if (!initialized_ || socket_fd_ < 0 || epoll_fd_ < 0) {
    std::cerr << "Interfaz no inicializada" << std::endl;
    return -1;
  }

  frames.clear();
  
  // Esperar por datos usando epoll con el tiempo especificado
  struct epoll_event events[1];
  int ready = epoll_wait(epoll_fd_, events, 1, timeout_ms);
  
  if (ready < 0) {
    if (errno != EINTR) {
      std::cerr << "Error en epoll_wait: " << strerror(errno) << std::endl;
    }
    return -1;
  }
  
  if (ready == 0) {
    // Timeout, no hay datos
    return 0;
  }

  // Imprimir el tipo de evento(s)
  // std::cout << "Eventos ocurridos: 0x" << std::hex << events[0].events << std::dec << std::endl;
  
  // Leer todos los mensajes disponibles en el buffer
  int frames_read = 0;
  while (true) {
    struct can_frame frame;
    ssize_t nbytes = ::read(socket_fd_, &frame, sizeof(struct can_frame));
    
    if (nbytes < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // No hay más datos disponibles
        break;
      } else {
        std::cerr << "Error al leer mensaje CAN: " << strerror(errno) << std::endl;
        return -1;
      }
    }
    
    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
      std::cerr << "Lectura incompleta de frame CAN" << std::endl;
      continue;
    }
    
    frames.push_back(frame);
    frames_read++;
  }
  
  return frames_read;
}

bool SocketCANInterface::write(const struct can_frame& frame) {
  if (!initialized_ || socket_fd_ < 0) {
    std::cerr << "Interfaz no inicializada" << std::endl;
    return false;
  }
  
  ssize_t nbytes = ::write(socket_fd_, &frame, sizeof(struct can_frame));
  
  if (nbytes < 0) {
    std::cerr << "Error al enviar mensaje CAN: " << strerror(errno) << std::endl;
    return false;
  }
  
  if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
    std::cerr << "Escritura incompleta de frame CAN" << std::endl;
    return false;
  }
  
  return true;
}