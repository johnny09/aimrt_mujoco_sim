#pragma once

#include "nlohmann/json.hpp"
namespace mc {

namespace io {
namespace axis {
namespace cmd_json {
using namespace nlohmann;

inline json get_aios_root_attribute_json = {{"method", "GET"},
                                            {"reqTarget", "/"}};
inline json enable_json = {
    {"method", "SET"}, {"reqTarget", "/m1/requested_state"}, {"property", 8}};
inline json disable_json = {
    {"method", "SET"}, {"reqTarget", "/m1/requested_state"}, {"property", 1}};
inline json set_control_mode_json = {{"method", "SET"},
                                     {"reqTarget", "/m1/controller/config"},
                                     {"control_mode", 1}};
inline json get_cvp_json = {{"method", "GET"}, {"reqTarget", "/m1/CVP"}};
inline json get_trapezoidal_trajectory_param_json = {
    {"method", "GET"}, {"reqTarget", "/m1/trap_traj"}};
inline json set_trapezoidal_trajectory_param_json = {
    {"method", "SET"},
    {"reqTarget", "/m1/trap_traj"},
    {"accel_limit", 20000},
    {"decel_limit", 20000},
    {"vel_limit", 60000}};
inline json get_motion_controller_config_json = {
    {"method", "GET"}, {"reqTarget", "/m1/controller/config"}};
inline json set_motion_controller_config_json = {
    {"method", "SET"},
    {"reqTarget", "/m1/controller/config"},
    {"pos_gain", 20},
    {"vel_gain", 0.0005},
    {"vel_integrator_gain", 0.0002},
    {"vel_limit", 400000},
    {"vel_limit_tolerance", 1.2}};
inline json move_to_json = {{"method", "SET"},
                            {"reqTarget", "/m1/trapezoidalMove"},
                            {"property", 0},
                            {"reply_enable", false}};
inline json set_linear_count_json = {
    {"method", "SET"}, {"reqTarget", "/m1/encoder"}, {"set_linear_count", 0}};
inline json ext_pos_json = {{"method", "SET"},
                            {"reqTarget", "/m1/setPosition"},
                            {"position", 0},
                            {"velocity_ff", 0},
                            {"current_ff", 0}};
inline json ext_vel_json = {{"method", "SET"},
                            {"reqTarget", "/m1/setVelocity"},
                            {"velocity", 0},
                            {"current_ff", 0}};
inline json ext_cur_json = {
    {"method", "SET"},
    {"reqTarget", "/m1/setCurrent"},
    {"current", 0},
};
inline json vel_ramp_enable_json = {
    {"method", "SET"},
    {"reqTarget", "/m1/controller"},
    {"vel_ramp_enable", true},
};
inline json set_vel_ramp_target_json = {
    {"method", "SET"},
    {"reqTarget", "/m1/controller"},
    {"vel_ramp_target", 0},
};
inline json get_motor_encoder_value_json = {{"method", "GET"},
                                            {"reqTarget", "/abs_encoder"}};
} // namespace cmd_json
} // namespace axis
} // namespace io
} // namespace mc
