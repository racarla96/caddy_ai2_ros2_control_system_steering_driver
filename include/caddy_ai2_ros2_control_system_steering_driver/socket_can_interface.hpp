#ifndef SOCKETCAN_INTERFACE_H
#define SOCKETCAN_INTERFACE_H

#include <string>
#include <vector>
#include <linux/can.h>
#include <sys/epoll.h>

class SocketCANInterface {
public:
  SocketCANInterface(const std::string& interface_name = "can0");
  ~SocketCANInterface();

  // Inicializar la interfaz CAN
  bool init();
  
  // Cerrar la interfaz CAN
  void close();
  
  // Leer mensajes CAN con tiempo de espera configurable
  int read(std::vector<struct can_frame>& frames, int timeout_ms = 100);
  
  // Enviar un mensaje CAN
  bool write(const struct can_frame& frame);
  
  // Comprobar si la interfaz está inicializada
  bool isInitialized() const { return initialized_; }

private:
  std::string interface_name_;
  int socket_fd_;
  int epoll_fd_;
  bool initialized_;
};

#endif // SOCKETCAN_INTERFACE_H