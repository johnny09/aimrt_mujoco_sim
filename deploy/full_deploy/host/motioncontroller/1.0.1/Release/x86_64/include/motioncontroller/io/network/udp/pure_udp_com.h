#pragma once
#include <netinet/in.h>
#include <unistd.h>

#include <string>

#include "motioncontroller/io/network/udp/udp_com.h"
#include "motioncontroller/io/network/udp/udp_com_config.h"
namespace mc {
namespace io {
namespace network {
class PureUDPCom : public UDPCom {
 public:
  PureUDPCom(std::string ip, uint16_t port);
  virtual int Send(const void* data, uint16_t length);
  virtual int Recv(void* data);
  virtual int Recv(void* data, std::string& ser_ip);

 private:
  int sfd_;
  struct sockaddr_in ser_addr_;
  struct sockaddr_in respond_ser_addr_;
  socklen_t respond_ser_addrlen_;
  std::string ser_ip_;
};
}  // namespace network
}  // namespace io
}  // namespace mc
