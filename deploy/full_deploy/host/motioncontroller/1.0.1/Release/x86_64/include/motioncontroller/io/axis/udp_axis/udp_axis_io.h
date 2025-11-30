#pragma once
#include "motioncontroller/data_type/axis_data.h"
#include "motioncontroller/function_type.h"
#include "motioncontroller/io/network/udp/udp_com.h"
#include "motioncontroller/io/network/udp/udp_com_config.h"
#include "motioncontroller/io/rtio.h"
#include <memory>
#include <mutex>
#include <string>
namespace mc {

namespace io {
namespace axis {
struct ExtPosPtCmd {
  float pos;
  short vel_ff;
  short cur_ff;
};
struct ExtVelPtCmd {
  float vel;
  float cur_ff;
};
struct ExtCurPtCmd {
  float current;
};
struct CvpPtFeedback {
  float pos_turn; // unit:turn
  float vel_turn; // unit:turn
  float cur;      // unit:A
  int reserved;
};
class UDPAxisIO : public RTIO {
public:
  UDPAxisIO(std::string ip);
  virtual void UpdateInput();
  virtual void UpdateOutput();
  data_type::AxisInput GetAxisInput();
  void SetAxisOutput(data_type::AxisOutput axis_output);

private:
  type::FunctionReturnType bin_to_cvp(void *bin, CvpPtFeedback &cvp_pt_fb);
  type::FunctionReturnType pos_cmd_to_bin(const ExtPosPtCmd &ext_pos_cmd,
                                          unsigned char *bin);

private:
  std::unique_ptr<io::network::UDPCom>
      pt_udp_com_uptr_; // pointer to non realtime udp communication manager

  char pt_recv_data_bin_[io::network::kMaxRecvByte] = {'\0'};
  double act_pos_turn_{0.0}, act_vel_turn_{0.0}, act_cur_{0.0};

  ExtPosPtCmd ext_pos_pt_cmd_;

  CvpPtFeedback cvp_pt_fb_;
  unsigned char pos_cmd_bin_[sizeof(ExtPosPtCmd) + 1] = {'\0'};
  unsigned char vel_cmd_bin_[sizeof(ExtVelPtCmd) + 1] = {'\0'};
  unsigned char cur_cmd_bin_[sizeof(ExtCurPtCmd) + 1] = {'\0'};
  unsigned char get_cvp_cmd_bin_[1] = {'\x1a'};

  bool enable_update_output_{false};
  double pos_offset_{0.0};

  data_type::AxisInput axis_input_;
  data_type::AxisInput axis_input_after_offset_;
  data_type::AxisOutput axis_output_;

  std::mutex rw_mtx_input_;
  std::mutex rw_mtx_output_;
};
} // namespace axis
} // namespace io
} // namespace mc