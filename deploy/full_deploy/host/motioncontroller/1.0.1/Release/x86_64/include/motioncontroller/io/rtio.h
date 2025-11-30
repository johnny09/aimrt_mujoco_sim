#pragma once
// #include "motioncontroller/data_type/axis_data.h"
namespace mc {
namespace io {
class RTIO {
 public:
  virtual void UpdateInput() = 0;
  virtual void UpdateOutput() = 0;
  // virtual data_type::AxisInput GetAxisInput() = 0;
  // virtual void SetAxisOutput(data_type::AxisOutput) = 0;

  //  protected:
};
}  // namespace io
}  // namespace mc
