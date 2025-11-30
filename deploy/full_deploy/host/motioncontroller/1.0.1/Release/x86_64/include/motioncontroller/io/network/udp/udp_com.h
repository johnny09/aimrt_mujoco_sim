#pragma once
#include <stdint.h>

#include <string>

#include "motioncontroller/io/network/udp/udp_com_config.h"
namespace mc {
namespace io {
namespace network {

class UDPCom {
 public:
  virtual int Send(const void* data, uint16_t length) = 0;
  virtual int Send(const std::string data);

  virtual int Recv(std::string& data, std::string& ser_ip);
  virtual int Recv(std::string& data);
  virtual int Recv(void* data) = 0;
  virtual int Recv(void* data, std::string& ser_ip) = 0;

 protected:
  char recv_data_bin_[io::network::kMaxRecvByte] = {'\0'};
  int ret_{0};
};
}  // namespace network
}  // namespace io
}  // namespace mc
