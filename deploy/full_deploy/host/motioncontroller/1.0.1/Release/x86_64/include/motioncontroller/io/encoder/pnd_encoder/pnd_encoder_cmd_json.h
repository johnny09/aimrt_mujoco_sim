
#pragma once
#include "nlohmann/json.hpp"
#include <cstdint>
namespace mc {
namespace encoder {
namespace pnd_encoder {
namespace cmd_json {
using namespace nlohmann;
inline json get_abs_encoder_value_json_0_0 = {{"id", "0"},
                                              {"method", "encoder.angle"}};
inline json get_abs_encoder_value_json_0_1 = {
    {"id", "1"}, {"method", "Encoder.Angle"}, {"params", ""}};

inline json get_motor_encoder_value_json = {{"method", "GET"},
                                            {"reqTarget", "/abs_encoder"}};
} // namespace cmd_json
} // namespace pnd_encoder
} // namespace encoder
} // namespace mc