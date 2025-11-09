// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "pid_control_module/pid_control_module.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "pid_control_module/global.h"
#include "yaml-cpp/yaml.h"

namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module {

bool PidControlModule::Initialize(aimrt::CoreRef core) {
  core_ = core;
  SetLogger(core_.GetLogger());

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();
    AIMRT_CHECK_ERROR_THROW(!file_path.empty(), "Can not get cfg file.");

    YAML::Node cfg_node = YAML::LoadFile(std::string(file_path));

    auto topic_name_sub = cfg_node["topic_name_sub"].as<std::string>();
    auto topic_name_pub = cfg_node["topic_name_pub"].as<std::string>();

    // Register joint state subscriber
    auto subscriber = core_.GetChannelHandle().GetSubscriber(topic_name_sub);
    AIMRT_CHECK_ERROR_THROW(subscriber, "Get subscriber for topic '{}' failed.", topic_name_sub);
    bool ret = aimrt::channel::Subscribe<aimrt::protocols::sensor::JointStateArray>(
        subscriber,
        std::bind(&PidControlModule::EventHandle, this, std::placeholders::_1));
    AIMRT_CHECK_ERROR_THROW(ret, "Subscribe failed.");

    // Register joint command publisher
    auto publisher = core_.GetChannelHandle().GetPublisher(topic_name_pub);
    AIMRT_CHECK_ERROR_THROW(publisher, "Get publisher for topic '{}' failed.", topic_name_pub);
    ret = aimrt::channel::RegisterPublishType<aimrt::protocols::sensor::JointCommandArray>(publisher);
    AIMRT_CHECK_ERROR_THROW(ret, "Register publishType failed.");
    publisher_proxy_ = std::make_unique<aimrt::channel::PublisherProxy<aimrt::protocols::sensor::JointCommandArray>>(publisher);

    // Register service
    service_ptr_ = std::make_unique<PidControlServiceImpl>(controller_);
    ret = core_.GetRpcHandle().RegisterService(service_ptr_.get());
    AIMRT_CHECK_ERROR_THROW(ret, "Register service failed.");

    // Init pid controller
    controller_.SetPIDParm(PIDParm{
        .Kp = cfg_node["init_kp"].as<double>(),
        .Ki = cfg_node["init_ki"].as<double>(),
        .Kd = cfg_node["init_kd"].as<double>()});

  } catch (const std::exception& e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool PidControlModule::Start() { return true; }

void PidControlModule::Shutdown() {}

void PidControlModule::EventHandle(const std::shared_ptr<const aimrt::protocols::sensor::JointStateArray>& data) {
  if (!data) {
    AIMRT_WARN("EventHandle received null JointStateArray");
    double control_output = 0.0;
    return;
  }

  AIMRT_INFO("Received JointStateArray:\n{}", data->DebugString());

  double control_output = 0.0;
  std::vector<std::string> joint_names{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  // 为每个 joint 生成一个 sin 波形的位置指令
  aimrt::protocols::sensor::JointCommandArray msg;
  auto now = std::chrono::steady_clock::now();
  double t = std::chrono::duration<double>(now.time_since_epoch()).count();

  const double amplitude = 0.5;  // 振幅 (rad)
  const double frequency = 0.5;  // 频率 (Hz)
  const double omega = 2.0 * 3.141592653589793 * frequency;

  for (int i = 0; i < data->joints_size(); ++i) {
    const auto& js = data->joints(i);
    auto* joint_cmd = msg.add_joints();
    joint_cmd->set_name(joint_names[i]);           // 使用输入的 joint 名称
    double phase = i * (3.141592653589793 / 4.0);  // 每个关节不同相位
    double position = amplitude * std::sin(omega * t + phase) - amplitude;
    joint_cmd->set_position(position);
  }

  publisher_proxy_->Publish(msg);
}

}  // namespace aimrt_mujoco_sim::example::inverted_pendulum::pid_control_module
