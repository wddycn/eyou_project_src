#include "eyou_ros2_control/eyou_system.hpp"
#include <unistd.h>
#include <iostream>
#include <limits>
#include <algorithm>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <map> // 引入 map 用于查找

namespace eyou_ros2_control
{

EyouSystem::EyouSystem() 
  : eyou_can_(nullptr)
{}

EyouSystem::~EyouSystem() {
    if (eyou_can_) {
        RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), "Shutting down Eyou motors...");
        for (auto id : motor_ids_) {
            eyou_can_->disableMotor(id);
        }
    }
}

hardware_interface::CallbackReturn EyouSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t num_joints = info_.joints.size();
  
  // 初始化缓冲区
  hw_states_.resize(num_joints * 3, 0.0);
  hw_commands_.resize(num_joints * 3, 0.0);
  motor_ids_.clear();
  
  // 预分配参数字向量
  joint_max_speeds_.resize(num_joints);
  joint_max_accels_.resize(num_joints);
  joint_max_torques_.resize(num_joints);

  // 读取全局 CAN 接口名称
  if (info_.hardware_parameters.find("can_interface") != info_.hardware_parameters.end()) {
      can_interface_name_ = info_.hardware_parameters.at("can_interface");
  } else {
      can_interface_name_ = "can1";
  }

  RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), "Initializing Eyou System with %zu joints", num_joints);

  // ============================================================
  // 【核心修改】在这里定义每个电机的专属参数 (无需改 URDF)
  // 格式: { 电机ID, {速度, 加速度, 力矩} }
  // 单位假设: rad/s, rad/s^2, Nm (请根据你的电机实际单位调整)
  // ============================================================
  struct MotorConfig {
      double speed;
      double accel;
      double torque;
  };

  // 这里根据你的实际电机 ID (2,3,4,5,6,7) 设置不同参数
  std::map<uint8_t, MotorConfig> id_config_map = {
      {2, {8.0,  4.0,  5.0}}, // 电机 2 (基座): 慢，稳，力气大
      {3, {8.0,  4.0,  5.0}},  // 电机 3: 中等
      {4, {8.0,  4.0,  5.0}},  // 电机 4: 中等
      {5, {8.0, 4.0,  5.0}},  // 电机 5: 较快
      {6, {8.0, 4.0,  2.0}},  // 电机 6: 快
      {7, {8.0, 4.0,  2.0}}   // 电机 7 (末端): 最快，力气小
  };

  // 默认配置 (如果 ID 不在上面的列表中，用这个)
  MotorConfig default_config = {2.0, 8.0, 5.0};

  for (size_t i = 0; i < num_joints; i++) {
    const auto & joint = info_.joints[i];
    
    // 1. 读取 Motor ID
    std::string param_name = joint.name + "_motor_id";
    if (info_.hardware_parameters.find(param_name) == info_.hardware_parameters.end()) {
      RCLCPP_FATAL(rclcpp::get_logger("EyouSystem"), "Hardware parameter '%s' not found", param_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    uint8_t id = static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at(param_name)));
    motor_ids_.push_back(id);

    // 2. 【自动匹配】根据 ID 从上面的 map 中查找参数
    MotorConfig cfg = default_config;
    if (id_config_map.find(id) != id_config_map.end()) {
        cfg = id_config_map[id];
    } else {
        RCLCPP_WARN(rclcpp::get_logger("EyouSystem"), "No specific config for Motor ID %d, using defaults.", id);
    }

    // 填入向量
    joint_max_speeds_[i] = cfg.speed;
    joint_max_accels_[i] = cfg.accel;
    joint_max_torques_[i] = cfg.torque;

    RCLCPP_INFO(
      rclcpp::get_logger("EyouSystem"), 
      "Joint[%zu] '%s' -> Motor ID: %d | Speed: %.2f | Accel: %.2f | Torque: %.2f",
      i, joint.name.c_str(), id, 
      joint_max_speeds_[i], joint_max_accels_[i], joint_max_torques_[i]
    );
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EyouSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const std::string & name = info_.joints[i].name;
    state_interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_states_[i * 3]);
    state_interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_states_[i * 3 + 1]);
    state_interfaces.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_states_[i * 3 + 2]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EyouSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const std::string & name = info_.joints[i].name;
    command_interfaces.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_commands_[i * 3]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn EyouSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), "Activating Eyou Motor Driver on interface: %s", can_interface_name_.c_str());

  try {
    eyou_can_ = std::make_unique<MotorCAN>(can_interface_name_);
    eyou_can_->bringUpCAN(1000000); 

    bool all_enabled = true;
    for (auto id : motor_ids_) {
      if (!eyou_can_->enableMotor(id)) {
        RCLCPP_ERROR(rclcpp::get_logger("EyouSystem"), "Failed to enable motor ID: %d", id);
        all_enabled = false;
      }
    }

    if (!all_enabled) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), "System Active.");
    return hardware_interface::CallbackReturn::SUCCESS;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("EyouSystem"), "Activation exception: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn EyouSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), "Deactivating Eyou Motor Driver...");
  if (eyou_can_) {
    for (auto id : motor_ids_) {
      eyou_can_->disableMotor(id);
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EyouSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!eyou_can_) return hardware_interface::return_type::ERROR;

  for (size_t i = 0; i < motor_ids_.size(); i++) {
    uint8_t id = motor_ids_[i];
    double pos = eyou_can_->readPosition(id);
    double vel = eyou_can_->readSpeed(id);
    double tor = eyou_can_->readTorque(id);

    hw_states_[i * 3]     = pos;
    hw_states_[i * 3 + 1] = vel;
    hw_states_[i * 3 + 2] = tor;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EyouSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!eyou_can_) return hardware_interface::return_type::ERROR;

  std::vector<uint8_t> ids = motor_ids_;
  std::vector<double> positions;
  std::vector<double> speeds;
  std::vector<float> torques; 
  std::vector<double> accels;
  std::vector<double> decels;

  size_t n = ids.size();
  positions.reserve(n);
  speeds.reserve(n);
  torques.reserve(n);
  accels.reserve(n);
  decels.reserve(n);

  for (size_t i = 0; i < n; i++) {
    positions.push_back(hw_commands_[i * 3]);
    
    // 使用我们在 on_init 中填好的独立参数
    speeds.push_back(joint_max_speeds_[i]);
    torques.push_back(static_cast<float>(joint_max_torques_[i]));
    accels.push_back(joint_max_accels_[i]);
    decels.push_back(joint_max_accels_[i]);
  }

  if (!eyou_can_->setProfilePositionMulti(ids, positions, speeds, torques, accels, decels)) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("EyouSystem"), 
      steady_clock, 
      1000, 
      "Command failed for one or more motors."
    );
  }

  return hardware_interface::return_type::OK;
}

}  // namespace eyou_ros2_control

PLUGINLIB_EXPORT_CLASS(eyou_ros2_control::EyouSystem, hardware_interface::SystemInterface)