#pragma once

namespace mc {
namespace data_type {
class AxisInput {
public:
  double act_pos;
  double act_vel;
  double act_cur;
};
class AxisOutput {
public:
  bool enable_update_rt_data;
  double set_pos;
  double set_vel;
  double set_cur;
};
} // namespace data_type
} // namespace mc
