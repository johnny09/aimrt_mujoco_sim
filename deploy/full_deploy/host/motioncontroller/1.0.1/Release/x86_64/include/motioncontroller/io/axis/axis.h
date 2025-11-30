#pragma once
#include "motioncontroller/function_type.h"
#include "motioncontroller/io/rtio.h"
#include <cmath>
#include <string>

// pre declare for friend class use
namespace mc {
namespace motion {
class AxisJog;
class AxisHalt;
class AxisMoveAbsolute;
class AxisMoveRelative;
class MoveJ;
} // namespace motion
} // namespace mc

namespace mc {
namespace io {
namespace axis {
enum class DriveMode { kDefault = 0, kPosMode, kVelMode, kCurMode };
enum class AxisType { kRotation = 0, kLinear };
enum class PowerState { kOn = 0, kOff };
class AxisState {
public:
  bool has_job{false};
  bool error{false};
};

// PD controller parameters
class PDParameter {
public:
  double kp{0.0};
  double kd{0.0};
};

// PID controller parameters
class PIDParameter {
public:
  double pp{0.0};
  double pi{0.0};
  double pd{0.0};
  double vp{0.0};
  double vi{0.0};
  double vd{0.0};
  double cp{0.0};
  double ci{0.0};
  double cd{0.0};
};

class Axis {
  // declare friend class
  friend class mc::motion::AxisJog;
  friend class mc::motion::AxisHalt;
  friend class mc::motion::AxisMoveAbsolute;
  friend class mc::motion::AxisMoveRelative;
  friend class mc::motion::MoveJ;

public:
  virtual type::FunctionReturnType Init(const bool execute) = 0;
  // reverse the direction of the axis relative to current direction
  virtual type::FunctionReturnType ReverseDir();
  // set the position of the axis to pos
  virtual type::FunctionReturnType SetPos(const double pos) = 0;
  // power on the axis
  virtual type::FunctionReturnType PowerOn(const bool execute) = 0;
  // power off the axis
  virtual type::FunctionReturnType PowerOff(const bool execute) = 0;

  // return the status to indicate whether the axis is moving or not
  virtual bool NotMoving() const;

  /*
  description: get the power state of the axis
  return:
    true:the axis is in powered on state
    false:the axis is powered off state
  */
  virtual bool IsPoweredOn() const = 0;
  // set the drive mode of the axis
  virtual type::FunctionReturnType SetDriveMode(DriveMode drive_mode) = 0;
  // get the drive mode of the axis
  virtual DriveMode GetDriveMode() const = 0;

  // set pd controller parameter
  virtual type::FunctionReturnType SetPD(const bool execute,
                                         const PDParameter pd_parameter) = 0;
  // set pid controller parameter
  virtual type::FunctionReturnType SetPID(const bool execute,
                                          const PIDParameter pid_parameter) = 0;

  /*
  description: set position to the axis in the unit of user defined
  params:
    pos: the position set to axis in the unit of user defined
    vel_ff: the velocity forward set to axis in the unit of user defined
    cur_ff: the current forward set to axis in the unit of user defined
  */
  virtual type::FunctionReturnType SetExtPos(const double pos,
                                             const double vel_ff = 0,
                                             const double cur_ff = 0) = 0;
  // virtual type::FunctionReturnType SetExtVel(const double vel, const double
  // cur_ff = 0) = 0; virtual type::FunctionReturnType SetExtCur(const double
  // cur) = 0;

  // update the actual position, actual velocity, actual current of the axis
  virtual void UpdateActPVC(double act_pos, double act_vel, double act_cur) = 0;

  // get the actual position of the axis in the unit of user defined
  virtual double GetActPos() const = 0;
  // get the actual velocity of the axis in the unit of user defined
  virtual double GetActVel() const = 0;
  // get the actual torque of the axis in the unit of user defined
  virtual double GetActTau() const = 0;

  // get the position value set to the axis in the unit of actuator
  virtual double GetExtSetPos() const = 0;
  // get the velocity value set to the axis in the unit of actuator
  virtual double GetExtSetVelFf() const = 0;
  // get the current value set to the axis in the unit of actuator
  virtual double GetExtSetCurFf() const = 0;

  // get the position value set to the axis in the unit of user defined
  virtual double GetSetPos() const = 0;
  // get the velocity forward value set to the axis in the unit of user defined
  virtual double GetSetVelFf() const = 0;
  // get the torque forward value set to the axis in the unit of user defined
  virtual double GetSetTauFf() const = 0;

  // set the gear ratio of the axis
  virtual type::FunctionReturnType SetGearRatio(double gear_ratio) = 0;
  // get the current gear ratio configuration of the axis
  virtual double GetGearRatio() const = 0;

  // set the minimum limit position of the axis in unit of user defined
  virtual type::FunctionReturnType set_min_pos(double min_pos);
  // set the maximum limit position of the axis in unit of user defined
  virtual type::FunctionReturnType set_max_pos(double max_pos);
  // set the maximum limit velocity of the axis in unit of user defined
  virtual type::FunctionReturnType set_max_vel(double max_vel);
  // set the maximum limit acceleration of the axis in unit of user defined
  virtual type::FunctionReturnType set_max_acc(double max_acc);
  // set the maximum limit deceleration of the axis in unit of user defined
  virtual type::FunctionReturnType set_max_dec(double max_dec);
  // set the maximum limit jerk of the axis in unit of user defined
  virtual type::FunctionReturnType set_max_jerk(double max_jerk);
  // get the minimum limit position of the axis in unit of user defined
  virtual double get_min_pos() const;
  // get the maximum limit position of the axis in unit of user defined
  virtual double get_max_pos() const;
  // get the maximum limit velocity of the axis in unit of user defined
  virtual double get_max_vel() const;
  // get the maximum limit acceleration of the axis in unit of user defined
  virtual double get_max_acc() const;
  // get the maximum limit deceleration of the axis in unit of user defined
  virtual double get_max_dec() const;
  // get the maximum limit jerk of the axis in unit of user defined
  virtual double get_max_jerk() const;

  // set the id of the axis
  virtual void set_id(std::string id);
  // get the id of the axis
  virtual std::string get_id() const;
  // set the name of the axis
  virtual void set_name(std::string name);
  // get the name of the axis
  virtual std::string get_name() const;

  // get the axis state
  virtual AxisState GetAxisState() const;

protected:
  void set_has_job();
  void clear_has_job();
  void set_error();
  void clear_error();

protected:
  std::string id_{""};
  std::string name_{"default"};
  int dir_{1};
  AxisType axis_type_{AxisType::kRotation};
  double gear_ratio_{1.0};
  double kt_{1.0}; // torque constant, Nm/A
  double max_vel_{M_PI_2};
  double max_acc_{2.0 * M_PI};
  double max_dec_{2.0 * M_PI};
  double max_jerk_{8.0 * M_PI};
  double min_pos_{-99999.0};
  double max_pos_{99999.0};

  AxisState axis_state_;

private:
  double zero_vel_threshold_{1e-3};
};
} // namespace axis
} // namespace io
} // namespace mc
