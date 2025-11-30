// Copyright (c) 2023, AgiBot Inc.
// All rights reserved.

#include "mujoco_sim_module/subscriber/joint_actuator_subscriber.h"

#include <cstddef>
#include "mujoco_sim_module/common/xmodel_reader.h"

namespace YAML {
template <>
struct convert<aimrt_mujoco_sim::mujoco_sim_module::subscriber::JointActuatorSubscriberBase::Options> {
  using Options = aimrt_mujoco_sim::mujoco_sim_module::subscriber::JointActuatorSubscriberBase::Options;
  static Node encode(const Options& rhs) {
    Node node;

    node["joints"] = YAML::Node();
    for (const auto& joint : rhs.joints) {
      Node joint_node;
      joint_node["name"] = joint.name;
      joint_node["bind_joint"] = joint.bind_joint;
      node["joints"].push_back(joint_node);
    }

    return node;
  }

  static bool decode(const Node& node, Options& rhs) {
    if (node["joints"] && node["joints"].IsSequence()) {
      for (const auto& joint_node : node["joints"]) {
        auto joint_node_options = Options::Joint{
            .name = joint_node["name"].as<std::string>(),
            .bind_joint = joint_node["bind_joint"].as<std::string>()};

        rhs.joints.emplace_back(std::move(joint_node_options));
      }
    }
    return true;
  }
};
}  // namespace YAML

namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber {
void JointActuatorSubscriberBase::SetMj(mjModel* m, mjData* d) {
  m_ = m;
  d_ = d;
}

void JointActuatorSubscriberBase::ApplyCtrlData() {
  // Calculate and update control value based on joint actuator type
  std::lock_guard<std::mutex> lock(command_array_mutex_);

  for (size_t i = 0; i < joint_num_ && i < joint_command_data_.size(); ++i) {
    if (joint_actuator_type_vec_[i] == "position") {
      current_command_array_[i] = joint_command_data_[i].position;
    } else if (joint_actuator_type_vec_[i] == "velocity") {
      current_command_array_[i] = joint_command_data_[i].velocity;
    } else if (joint_actuator_type_vec_[i] == "motor") {
      // motor: calculate based on current state
      double state_position = d_->qpos[actuator_bind_joint_sensor_addr_vec_[i].pos_addr];
      double state_velocity = d_->qvel[actuator_bind_joint_sensor_addr_vec_[i].vel_addr];

      current_command_array_[i] = joint_command_data_[i].effort +
                                  joint_command_data_[i].stiffness * (joint_command_data_[i].position - state_position) +
                                  joint_command_data_[i].damping * (joint_command_data_[i].velocity - state_velocity);
    } else {
      AIMRT_WARN("Invalid joint actuator type '{}'.", joint_actuator_type_vec_[i]);
      current_command_array_[i] = 0.0;
    }

    // Update control value
    d_->ctrl[actuator_addr_vec_[i]] = current_command_array_[i];
  }

  if (joint_num_ > 0) {
    // AIMRT_INFO("!!!!!!!joint_id: 0, command_pos: {}", d_->ctrl[actuator_addr_vec_[0]]);
  }
}

void JointActuatorSubscriberBase::RegisterActuatorAddr() {
  for (auto const& joint : options_.joints) {
    int32_t actuator_id = common::GetJointActIdByJointName(m_, joint.bind_joint).value_or(-1);

    actuator_addr_vec_.emplace_back(actuator_id);
    joint_names_vec_.emplace_back(joint.name);
    joint_actuator_type_vec_.emplace_back(common::GetJointActTypeByJointName(m_, joint.bind_joint).value_or(""));

    auto joint_id = m_->actuator_trnid[static_cast<int32_t>(actuator_id * 2)];
    actuator_bind_joint_sensor_addr_vec_.emplace_back(ActuatorBindJointSensorAddr{
        .pos_addr = m_->jnt_qposadr[joint_id],
        .vel_addr = m_->jnt_dofadr[joint_id],
    });
  }

  joint_num_ = actuator_addr_vec_.size();
  // Initialize command arrays with zeros
  current_command_array_.resize(joint_num_, 0.0);
  joint_command_data_.resize(joint_num_);
}

void JointActuatorSubscriberBase::InitializeBase(YAML::Node options_node) {
  if (options_node && !options_node.IsNull())
    options_ = options_node.as<Options>();

  RegisterActuatorAddr();

  options_node = options_;
}

void JointActuatorSubscriber::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);

  AIMRT_CHECK_ERROR_THROW(aimrt::channel::Subscribe<aimrt::protocols::sensor::JointCommandArray>(
                              subscriber_,
                              std::bind(&JointActuatorSubscriber::EventHandle, this, std::placeholders::_1)),
                          "Subscribe failed.");
}

void JointActuatorSubscriber::EventHandle(const std::shared_ptr<const aimrt::protocols::sensor::JointCommandArray>& commands) {
  if (stop_flag_) [[unlikely]]
    return;

  // AIMRT_INFO("JointActuatorSubscriber::EventHandle, commands: {}", aimrt::Pb2CompactJson(*commands));
  size_t joint_num_real = commands->joints_size();

  if (joint_num_ != joint_num_real) [[unlikely]] {
    AIMRT_WARN("The number of joints (topci: {}, expected: {}, actual: {}) in the options and the received message are different.",
               subscriber_.GetTopic(), joint_num_, joint_num_real);
    joint_num_real = std::min(joint_num_, joint_num_real);
  }

  // Save command data only, calculation will be done in ApplyCtrlData
  std::lock_guard<std::mutex> lock(command_array_mutex_);

  for (size_t ii = 0; ii < joint_num_real; ++ii) {
    const auto& command = commands->joints()[ii];

    auto itr = std::ranges::find(joint_names_vec_, command.name());
    if (itr == joint_names_vec_.end()) [[unlikely]] {
      AIMRT_WARN("Invalid msg for topic '{}', msg: {}, maybe the joint name : {} is not matched.",
                 subscriber_.GetTopic(), aimrt::Pb2CompactJson(*commands), command.name());
      continue;
    }
    uint32_t joint_idx = std::distance(joint_names_vec_.begin(), itr);

    // Save raw command data
    joint_command_data_[joint_idx].position = command.position();
    joint_command_data_[joint_idx].velocity = command.velocity();
    joint_command_data_[joint_idx].effort = command.effort();
    joint_command_data_[joint_idx].stiffness = command.stiffness();
    joint_command_data_[joint_idx].damping = command.damping();
  }

  // if (joint_num_real > 0) {
  //   AIMRT_INFO("joint_id: 0, command_pos: {}, command_vel: {}, command_effort: {}, command_stiffness: {}, command_damping: {}",
  //              commands->joints()[0].position(), commands->joints()[0].velocity(),
  //              commands->joints()[0].effort(), commands->joints()[0].stiffness(),
  //              commands->joints()[0].damping());
  // }
}

#ifdef AIMRT_MUJOCO_SIM_BUILD_WITH_ROS2
void JointActuatorRos2Subscriber::Initialize(YAML::Node options_node) {
  InitializeBase(options_node);

  AIMRT_CHECK_ERROR_THROW(aimrt::channel::Subscribe<aimrt_msgs::msg::JointCommandArray>(
                              subscriber_,
                              std::bind(&JointActuatorRos2Subscriber::EventHandle, this, std::placeholders::_1)),
                          "Subscribe failed.");
}
void JointActuatorRos2Subscriber::EventHandle(const std::shared_ptr<const aimrt_msgs::msg::JointCommandArray>& commands) {
  if (stop_flag_) [[unlikely]]
    return;

  size_t joint_num_real = commands->joints.size();

  if (joint_num_ != joint_num_real) [[unlikely]] {
    AIMRT_WARN("The number of joints (topci: {}, expected: {}, actual: {}) in the options and the received message are different.",
               subscriber_.GetTopic(), joint_num_, joint_num_real);
    joint_num_real = std::min(joint_num_, joint_num_real);
  }

  // Save command data only, calculation will be done in ApplyCtrlData
  std::lock_guard<std::mutex> lock(command_array_mutex_);

  for (size_t ii = 0; ii < joint_num_real; ++ii) {
    const auto command = commands->joints[ii];

    auto itr = std::ranges::find(joint_names_vec_, command.name);
    if (itr == joint_names_vec_.end()) [[unlikely]] {
      AIMRT_WARN("Invalid msg for topic '{}', msg: {}, Joint name '{}' is not matched.",
                 subscriber_.GetTopic(), aimrt_msgs::msg::to_yaml(*commands), command.name);
      continue;
    }
    uint32_t joint_idx = std::distance(joint_names_vec_.begin(), itr);

    // Save raw command data
    joint_command_data_[joint_idx].position = command.position;
    joint_command_data_[joint_idx].velocity = command.velocity;
    joint_command_data_[joint_idx].effort = command.effort;
    joint_command_data_[joint_idx].stiffness = command.stiffness;
    joint_command_data_[joint_idx].damping = command.damping;
  }
}
#endif
}  // namespace aimrt_mujoco_sim::mujoco_sim_module::subscriber