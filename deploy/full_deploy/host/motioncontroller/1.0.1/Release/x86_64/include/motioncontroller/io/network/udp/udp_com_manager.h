#pragma once
#include <memory>
#include <string>

#include "motioncontroller/core/Time.hpp"
#include "motioncontroller/function_type.h"
#include "motioncontroller/io/network/udp/udp_com.h"
#include "nlohmann/json.hpp"
namespace mc {
namespace io {
namespace network {

class UDPComManager {
 public:
  UDPComManager(const std::string ip, const uint16_t port);
  type::FunctionReturnType Send(const bool execute, const nlohmann::json cmd, const double timeout_ms,
                                nlohmann::json& fb);
  type::FunctionReturnType Send(const bool execute, const nlohmann::json cmd, const double timeout_ms);
  type::FunctionReturnType Send(const bool execute, const std::string cmd, const double timeout_ms, std::string fb);

 private:
  enum class State { kIdle = 0, kWaitFb, kDone, kError };
  State state_{State::kIdle};
  nlohmann::json cmd_;
  std::shared_ptr<io::network::UDPCom> nrt_udp_com_sptr_;  //
  broccoli::core::Time start_udp_socket_time_{0, 0};
};
}  // namespace network
}  // namespace io
}  // namespace mc