#include "joy_stick_module/joy_stick_module.h"

#include "aimrt_module_protobuf_interface/channel/protobuf_channel.h"
#include "aimrt_module_protobuf_interface/util/protobuf_tools.h"
#include "joystick_cmd.pb.h"
#include "motioncontroller/ui/joystick/xbox/xbox_joy.h"
#include "yaml-cpp/yaml.h"

bool JoyStickModule::Initialize(aimrt::CoreRef core) {
  // Save aimrt framework handle
  core_ = core;

  try {
    // Read cfg
    auto file_path = core_.GetConfigurator().GetConfigFilePath();

    if (!file_path.empty()) {
      YAML::Node cfg_node = YAML::LoadFile(file_path.data());
      control_freq_ = cfg_node["pub_freq"].as<int32_t>();
      topic_name_ = cfg_node["pub_topic"].as<std::string>();

      // init joystick
      if (!InitJoyStick(cfg_node)) {
        return false;
      }
    } else
      AIMRT_WARN("cfg file path is empty.");

    // Get executor handle
    executor_ = core_.GetExecutorManager().GetExecutor("joy_thread_executor");
    AIMRT_CHECK_ERROR_THROW(executor_, "Get executor 'joy_thread_executor' failed.");
    read_joy_data_executor_ = core_.GetExecutorManager().GetExecutor("read_joy_data_executor");
    AIMRT_CHECK_ERROR_THROW(read_joy_data_executor_,
                            "Get executor 'read_joy_data_executor' failed.");

    // Prepare Publisher
    pub_joystick_cmd_ = core_.GetChannelHandle().GetPublisher(topic_name_);
    aimrt::channel::RegisterPublishType<my_proto::joystick_cmd>(pub_joystick_cmd_);
    AIMRT_CHECK_ERROR_THROW(pub_joystick_cmd_, "Create publisher {} failed.", topic_name_);

  } catch (const std::exception &e) {
    AIMRT_ERROR("Init failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Init succeeded.");

  return true;
}

bool JoyStickModule::Start() {
  try {
    run_flag_ = true;
    auto period = std::chrono::microseconds(1'000'000 / control_freq_);
    timer_ = aimrt::executor::CreateTimer(executor_, period, [this]() { this->MainLoop(); });

    read_joy_data_executor_.Execute(std::bind(&JoyStickModule::ReadJoyStickData, this));
  } catch (const std::exception &e) {
    AIMRT_ERROR("Start failed, {}", e.what());
    return false;
  }

  AIMRT_INFO("Start succeeded.");
  return true;
}

void JoyStickModule::MainLoop() {
  static aimrt::channel::PublisherProxy<my_proto::joystick_cmd> pub_joystick_cmd(pub_joystick_cmd_);
  // if (!run_flag_) return;
  try {
    my_proto::joystick_cmd joy_cmd_msg;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // for (const auto &[key, value] : joystick_data_.key_value_map) {
      //   // AIMRT_DEBUG("key: {}, value: {}", key, value);
      //   joy_cmd_msg.add_key_value(value);
      // }
      for (const auto &key_name : xbox_key_names_) {
        auto it = joystick_data_.key_value_map.find(key_name);
        if (it != joystick_data_.key_value_map.end()) {
          joy_cmd_msg.add_key_value(it->second);
        } else {
          // If key not found, push a default value (e.g., 0.0)
          joy_cmd_msg.add_key_value(0.0);
        }
      }
    }

    pub_joystick_cmd.Publish(joy_cmd_msg);

  } catch (const std::exception &e) {
    AIMRT_ERROR("Exit MainLoop with exception, {}", e.what());
  }
}

void JoyStickModule::ReadJoyStickData() {
  while (run_flag_) {
    try {
      mc::ui::joystick::JoyStickData joystick_data_tmp;
      joystick_data_tmp = joystick_ptr_->GetInputData();
      {
        std::lock_guard<std::mutex> lock(mutex_);
        joystick_data_ = joystick_data_tmp;
      }
    } catch (const std::exception &e) {
      AIMRT_ERROR("Read joystick data failed, {}", e.what());
    }
  }
}

void JoyStickModule::Shutdown() {
  run_flag_.store(false);
  timer_->Cancel();
  AIMRT_INFO("Shutdown succeeded.");
}

bool JoyStickModule::InitJoyStick(YAML::Node &cfg_node) {
  if (!cfg_node["joystick"]) {
    AIMRT_ERROR("No joystick config found!");
    return false;
  }
  auto joy_node = cfg_node["joystick"];

  std::string io_name = joy_node["io_name"].as<std::string>();
  std::string type = joy_node["type"].as<std::string>();
  std::string connect_type = joy_node["connect_type"].as<std::string>();

  if (type == "xbox") {
    try {
      joystick_ptr_ = std::make_shared<mc::ui::joystick::XboxJoyStick>(
          io_name.c_str(), (connect_type == "bluetooth"));
    } catch (const std::exception &e) {
      AIMRT_ERROR("Init joystick {} failed, {}", io_name, e.what());
      return false;
    }
    AIMRT_INFO("Init joystick {} as XboxJoy", io_name);
  } else {
    AIMRT_ERROR("Joystick type {} not support!", type);
    return false;
  }
  return true;
}