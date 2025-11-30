#pragma once
#include "motioncontroller/function_type.h"
namespace mc {
namespace encoder {
class Encoder {
public:
  virtual mc::type::FunctionReturnType GetPos(const bool execute,
                                              double &pos) const = 0;
};
} // namespace encoder
} // namespace mc