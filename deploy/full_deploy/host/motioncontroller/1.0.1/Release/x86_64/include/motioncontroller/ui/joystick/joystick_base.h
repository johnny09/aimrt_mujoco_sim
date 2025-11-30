#pragma once
#include "motioncontroller/function_type.h"
#include <string>
#include <unordered_map>
namespace mc {
namespace ui {

namespace joystick {

struct JoyStickData {
  double time{0.0}; // time stamp in second
  // key name and value
  // button name and state: pressed 1, released 0
  // axis name and value: -100 ~ 100
  std::unordered_map<std::string, int16_t> key_value_map; // key name and value
};
class JoyStickBase {
public:
  JoyStickBase(const char *dev_name);
  ~JoyStickBase();
  // mc::type::FunctionReturnType InitDev();
  virtual JoyStickData GetInputData() = 0;

protected:
  int dev_fd_{0};
  char dev_name_[4];

private:
  char dev_path_[12] = "/dev/input/";
  int dev_open(const char *dev_name);
  int dev_close(const int dev_fd);
};
} // namespace joystick

} // namespace ui
} // namespace mc
