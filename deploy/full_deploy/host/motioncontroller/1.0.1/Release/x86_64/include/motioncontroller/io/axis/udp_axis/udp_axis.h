#pragma once
#include <memory>

#include "motioncontroller/io/axis/axis.h"
#include "motioncontroller/io/network/udp/udp_com.h"
#include "motioncontroller/io/network/udp/udp_com_config.h"
#include "motioncontroller/io/network/udp/udp_com_manager.h"

namespace mc {

namespace io {
namespace axis {
class ActuatorAttribute {
public:
  std::string serial_number;
  std::string connect_mode;
  std::string name;
  std::string model;
  int vbus_voltage;
  int motor_temp_m1;
  int inverter_temp_m1;
  std::vector<std::string> hw_version;
  std::vector<std::string> fw_version;
};
class UDPAxis : public Axis {
public:
  UDPAxis() = delete; // default constructor
  UDPAxis(std::string ip, double kt);
  virtual type::FunctionReturnType Init(const bool execute) override;
  virtual type::FunctionReturnType SetPos(const double pos) override;
  virtual type::FunctionReturnType PowerOn(const bool execute) override;
  virtual type::FunctionReturnType PowerOff(const bool execute) override;
  virtual bool IsPoweredOn() const override;

  virtual type::FunctionReturnType SetDriveMode(DriveMode drive_mode) override;
  virtual DriveMode GetDriveMode() const override;

  // set pd controller parameter
  virtual type::FunctionReturnType
  SetPD(const bool execute, const PDParameter pd_parameter) override;
  // set pid controller parameter
  virtual type::FunctionReturnType
  SetPID(const bool execute, const PIDParameter pid_parameter) override;

  virtual type::FunctionReturnType SetExtPos(const double pos,
                                             const double vel_ff = 0,
                                             const double tau_ff = 0) override;

  virtual void UpdateActPVC(double act_pos, double act_vel,
                            double act_cur) override;
  mc::type::FunctionReturnType GetMotorEnc(const bool execute,
                                           double &pos) const;
  virtual double GetActPos() const override;
  virtual double GetActVel() const override;
  virtual double GetActTau() const override;
  virtual double GetExtSetPos() const override;
  virtual double GetExtSetVelFf() const override;
  virtual double GetExtSetCurFf() const override;

  virtual double GetSetPos() const override;
  virtual double GetSetVelFf() const override;
  virtual double GetSetTauFf() const override;

  virtual type::FunctionReturnType SetGearRatio(double gear_ratio);
  virtual double GetGearRatio() const;

private:
  void set_power_state(const PowerState state);
  PowerState get_power_state() const;

private:
  // AxisType axis_type_{AxisType::kRotation};
  double pos_offset_{0.0};

  std::unique_ptr<io::network::UDPComManager> async_json_cmd_mgr_uptr_;
  PowerState power_state_{PowerState::kOff};
  ActuatorAttribute act_attr_;

  double act_pos_without_offset_{0.0};
  double act_pos_{0.0}, act_vel_{0.0}, act_tau_{0.0};

  double ext_set_pos_{0.0}, ext_set_vel_ff_{0.0}, ext_set_cur_ff_{0.0};
  double set_pos_{0.0}, set_vel_ff_{0.0}, set_tau_ff_{0.0};

  double axis_io_pos_{0.0};
  double axis_io_pos_cmd_offset_{0.0};
  double axis_io_pos_for_user_set_pos_{0.0}; // the init axis io position
  double user_set_pos_{0.0};

  DriveMode drive_mode_{
      DriveMode::kDefault}; // default drive mode is ext pos mode
};
} // namespace axis
} // namespace io
} // namespace mc