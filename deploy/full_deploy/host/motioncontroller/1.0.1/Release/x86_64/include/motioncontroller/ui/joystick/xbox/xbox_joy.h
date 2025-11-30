#pragma once
#include "motioncontroller/ui/joystick/joystick_base.h"
#include <string>
#include <variant>
#include <vector>
namespace mc {
namespace ui {
namespace joystick {

// enum class ButtonKeyMapA {
//   XBOX_BUTTON_A = 0x00,
//   XBOX_BUTTON_B = 0x01,
//   XBOX_BUTTON_X = 0x03,
//   XBOX_BUTTON_Y = 0x04,
//   XBOX_BUTTON_LB = 0x06,
//   XBOX_BUTTON_RB = 0x07,
//   XBOX_BUTTON_START = 0x0b,
//   XBOX_BUTTON_BACK = 0x05,
//   XBOX_BUTTON_HOME = 0x02,
//   XBOX_BUTTON_SEL = 0x0a,
//   XBOX_BUTTON_LO = 0x0d,
//   XBOX_BUTTON_RO = 0x0e
// };

// enum class ButtonKeyMapB {
//   XBOX_BUTTON_A = 0x00,
//   XBOX_BUTTON_B = 0x01,
//   XBOX_BUTTON_X = 0x02,
//   XBOX_BUTTON_Y = 0x03,
//   XBOX_BUTTON_LB = 0x04,
//   XBOX_BUTTON_RB = 0x05,
//   XBOX_BUTTON_START = 0x07,
//   XBOX_BUTTON_BACK = 0x06,
//   XBOX_BUTTON_HOME = 0x08,
//   XBOX_BUTTON_LO = 0x09,
//   XBOX_BUTTON_RO = 0x0a
// };

#define XBOX_BUTTON_ON 0x01
#define XBOX_BUTTON_OFF 0x00
#define XBOX_AXIS_VAL_UP -32767
#define XBOX_AXIS_VAL_DOWN 32767
#define XBOX_AXIS_VAL_LEFT -32767
#define XBOX_AXIS_VAL_RIGHT 32767

#define XBOX_AXIS_VAL_MIN -32767
#define XBOX_AXIS_VAL_MAX 32767
#define XBOX_AXIS_VAL_MID 0x00

std::vector<std::string> xbox_key_names = {
    "A",  "B",  "X",  "Y",  "LB", "RB", "START", "BACK", "HOME", "LO",
    "RO", "LX", "LY", "RX", "RY", "LT", "RT",    "XX",   "YY"};

class XboxJoyStick : public JoyStickBase {
public:
  XboxJoyStick(const char *dev_name, const bool bluetooth = true);
  // mc::type::FunctionReturnType InitDev();
  JoyStickData GetInputData() override;

private:
  std::unordered_map<std::string, int> button_map_;
  JoyStickData joystick_data_;
};

} // namespace joystick
} // namespace ui
} // namespace mc