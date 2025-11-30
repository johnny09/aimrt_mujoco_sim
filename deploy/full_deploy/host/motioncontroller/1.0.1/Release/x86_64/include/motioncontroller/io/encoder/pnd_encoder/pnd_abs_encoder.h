#pragma once
#include "motioncontroller/function_type.h"
#include "motioncontroller/io/encoder/encoder.h"
#include "motioncontroller/io/network/udp/udp_com_manager.h"
#include <memory>
#include <string>
namespace mc {
namespace encoder {
class PNDAbsEncoder_0_0 : public Encoder {
public:
  PNDAbsEncoder_0_0(std::string ip);
  virtual mc::type::FunctionReturnType GetPos(const bool execute,
                                              double &pos) const;

private:
  nlohmann::json json_fb_;
  std::string ip_;
  std::unique_ptr<mc::io::network::UDPComManager> async_json_cmd_mgr_uptr_;
};

class PNDAbsEncoder_0_1 : public Encoder {
public:
  PNDAbsEncoder_0_1(std::string ip);
  virtual mc::type::FunctionReturnType GetPos(const bool execute,
                                              double &pos) const;

private:
  nlohmann::json json_fb_;
  std::string ip_;
  std::unique_ptr<mc::io::network::UDPComManager> async_json_cmd_mgr_uptr_;
};
} // namespace encoder
} // namespace mc