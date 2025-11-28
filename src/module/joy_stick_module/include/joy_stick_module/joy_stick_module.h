#pragma once

#include <iostream>
#include "aimrt_module_cpp_interface/executor/timer.h"
#include "aimrt_module_cpp_interface/module_base.h"
#include "motioncontroller/ui/joystick/joystick_base.h"
#include "yaml-cpp/yaml.h"
class JoyStickModule : public aimrt::ModuleBase {
 public:
  JoyStickModule() = default;
  ~JoyStickModule() override = default;

  aimrt::ModuleInfo Info() const override { return aimrt::ModuleInfo{.name = "JoyStickModule"}; }

  bool Initialize(aimrt::CoreRef core) override;
  bool Start() override;
  void Shutdown() override;

  bool InitJoyStick(YAML::Node& cfg_node);
  void ReadJoyStickData();

 private:
  auto GetLogger() { return core_.GetLogger(); }
  void MainLoop();

 private:
  aimrt::CoreRef core_;
  aimrt::executor::ExecutorRef executor_;
  aimrt::executor::ExecutorRef read_joy_data_executor_;
  std::shared_ptr<aimrt::executor::TimerBase> timer_;

  std::atomic_bool run_flag_ = false;
  uint32_t control_freq_ = 400;  // Hz

  std::string topic_name_ = "";

  // channel
  aimrt::channel::PublisherRef pub_joystick_cmd_;

  // joystick
  std::shared_ptr<mc::ui::joystick::JoyStickBase> joystick_ptr_;
  mc::ui::joystick::JoyStickData joystick_data_;

  // joystick cmd msg order
  std::vector<std::string> xbox_key_names_ = {"A",    "B",    "X",  "Y",  "LB", "RB", "START",
                                              "BACK", "HOME", "LO", "RO", "LX", "LY", "RX",
                                              "RY",   "LT",   "RT", "XX", "YY"};

  uint32_t freq_{};

  std::mutex mutex_;
};