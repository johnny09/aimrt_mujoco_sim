#pragma once
namespace mc {

namespace io {
namespace axis {
namespace config {

// #define MOTOR_ENABLE_TIMEOUT_TIME_MS 200
inline const uint16_t kRTPort = 2333;
inline const uint16_t kNRTPort = 2334;
inline const uint16_t kPTPort = 10000;
inline const double kMotorEnableTimeoutTimeMs = 200;
inline const double kMotorDisableTimeoutTimeMs = 200;
inline const double kActuatorInitTimeoutTimeMs = 200;
inline const uint16_t kEncoderReadTimeoutTimeMs = 10;
inline const double kCountPerTurn = 4000;
} // namespace config
} // namespace axis
} // namespace io

} // namespace mc
