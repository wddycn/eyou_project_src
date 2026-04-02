#include "eyou_test_ros2_control/eyou_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace eyou_test_ros2_control
{
// 初始化硬件
hardware_interface::CallbackReturn EyouSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // 先调用父类初始化
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try
  {
    // 读取CAN接口名称（默认can0）
    auto can_it = info.hardware_parameters.find("can_interface");
    if (can_it != info.hardware_parameters.end())
    {
      can_interface_ = can_it->second;
    }
    else
    {
      can_interface_ = "can0"; // 默认值
    }

    // 读取电机ID列表（必须配置）
    auto motor_ids_it = info.hardware_parameters.find("motor_ids");
    if (motor_ids_it == info.hardware_parameters.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("EyouSystem"), "未配置motor_ids参数！");
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::string motor_ids_str = motor_ids_it->second;

    // 解析格式："13,10"
    size_t pos = 0;
    while ((pos = motor_ids_str.find(",")) != std::string::npos)
    {
      motor_ids_.push_back(std::stoi(motor_ids_str.substr(0, pos)));
      motor_ids_str.erase(0, pos + 1);
    }
    motor_ids_.push_back(std::stoi(motor_ids_str));
    num_motors_ = motor_ids_.size();

    // 初始化数据容器
    hw_positions_.resize(num_motors_, 0.0);
    hw_velocities_.resize(num_motors_, 0.0);
    hw_torques_.resize(num_motors_, 0.0);
    hw_commands_position_.resize(num_motors_, 0.0);
    hw_commands_velocity_.resize(num_motors_, 0.0);
    hw_commands_torque_.resize(num_motors_, 0.0);

    // ========== 新增：解析每个电机的独立运动参数 ==========
    motor_speeds_.resize(num_motors_, 3.0);    // 默认速度3rad/s
    motor_torques_.resize(num_motors_, 1.0f);  // 默认力矩1Nm
    motor_accels_.resize(num_motors_, 4.0);    // 默认加速度4rad/s²
    motor_decels_.resize(num_motors_, 4.0);    // 默认减速度4rad/s²

    // 为每个电机读取独立参数
    for (size_t i = 0; i < num_motors_; i++)
    {
      uint8_t motor_id = motor_ids_[i];
      std::string prefix = "motor_" + std::to_string(motor_id);

      // 读取速度参数
      auto speed_it = info.hardware_parameters.find(prefix + "_speed");
      if (speed_it != info.hardware_parameters.end())
      {
        motor_speeds_[i] = std::stod(speed_it->second);
      }

      // 读取力矩参数
      auto torque_it = info.hardware_parameters.find(prefix + "_torque");
      if (torque_it != info.hardware_parameters.end())
      {
        motor_torques_[i] = std::stof(torque_it->second);
      }

      // 读取加速度参数
      auto accel_it = info.hardware_parameters.find(prefix + "_accel");
      if (accel_it != info.hardware_parameters.end())
      {
        motor_accels_[i] = std::stod(accel_it->second);
      }

      // 读取减速度参数
      auto decel_it = info.hardware_parameters.find(prefix + "_decel");
      if (decel_it != info.hardware_parameters.end())
      {
        motor_decels_[i] = std::stod(decel_it->second);
      }

      // 打印每个电机的参数
      RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), 
        "电机 %d 参数：速度=%.1f rad/s，力矩=%.1f Nm，加速度=%.1f rad/s²，减速度=%.1f rad/s²",
        motor_id, motor_speeds_[i], motor_torques_[i], motor_accels_[i], motor_decels_[i]);
    }

    // 创建Eyou CAN实例
    eyou_can_ = std::make_unique<MotorCAN>(can_interface_);
    // 初始化CAN总线（只需要初始化一次）
    // eyou_can_->bringUpCAN(1000000);

    RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), "初始化成功，CAN接口：%s，电机ID：", can_interface_.c_str());
    for (auto id : motor_ids_)
    {
      RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), " %d", id);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EyouSystem"), "初始化失败：%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

// 导出状态接口
std::vector<hardware_interface::StateInterface> EyouSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < num_motors_; i++)
  {
    // 位置状态接口
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]));

    // 速度状态接口
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]));

    // 力矩状态接口
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_EFFORT,  // effort对应力矩
      &hw_torques_[i]));
  }

  return state_interfaces;
}

// 导出命令接口
std::vector<hardware_interface::CommandInterface> EyouSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < num_motors_; i++)
  {
    // 位置命令接口（默认使用位置模式）
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_position_[i]));

    // 可选：速度命令接口
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &hw_commands_velocity_[i]));

    // 可选：力矩命令接口
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name,
      hardware_interface::HW_IF_EFFORT,
      &hw_commands_torque_[i]));
  }

  return command_interfaces;
}

// 激活硬件（使能电机）
hardware_interface::CallbackReturn EyouSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    bool all_enabled = true;
    for (size_t i = 0; i < num_motors_; i++)
    {
      bool enabled = eyou_can_->enableMotor(motor_ids_[i]);
      if (!enabled)
      {
        RCLCPP_ERROR(rclcpp::get_logger("EyouSystem"), "电机 %d 使能失败", motor_ids_[i]);
        all_enabled = false;
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), "电机 %d 使能成功", motor_ids_[i]);
      }
    }

    if (all_enabled)
    {
      // 重置命令和状态
      for (size_t i = 0; i < num_motors_; i++)
      {
        hw_commands_position_[i] = 0.0;
        hw_commands_velocity_[i] = 0.0;
        hw_commands_torque_[i] = 0.0;
        hw_positions_[i] = 0.0;
        hw_velocities_[i] = 0.0;
        hw_torques_[i] = 0.0;
      }
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    else
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EyouSystem"), "激活失败：%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

// 停用硬件（失能电机）
hardware_interface::CallbackReturn EyouSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    for (size_t i = 0; i < num_motors_; i++)
    {
      eyou_can_->disableMotor(motor_ids_[i]);
      RCLCPP_INFO(rclcpp::get_logger("EyouSystem"), "电机 %d 失能成功", motor_ids_[i]);
    }
    // eyou_can_->bringDownCAN();  // 最后一次运行时调用
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EyouSystem"), "停用失败：%s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
}

// 读取电机状态
hardware_interface::return_type EyouSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try
  {
    for (size_t i = 0; i < num_motors_; i++)
    {
      // 读取位置（rad）
      hw_positions_[i] = eyou_can_->readPosition(motor_ids_[i]);
      // 读取速度（rad/s）
      hw_velocities_[i] = eyou_can_->readSpeed(motor_ids_[i]);
      // 读取力矩（Nm）
      hw_torques_[i] = eyou_can_->readTorque(motor_ids_[i]);
    }
    return hardware_interface::return_type::OK;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EyouSystem"), "读取状态失败：%s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

// 写入控制命令
hardware_interface::return_type EyouSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  try
  {
    // 位置轮廓模式（多电机）- 使用每个电机的独立参数
    std::vector<uint8_t> ids = motor_ids_;
    std::vector<double> positions = hw_commands_position_;
    
    // ========== 关键修改：使用独立参数而非统一值 ==========
    std::vector<double> speeds = motor_speeds_;        // 每个电机的独立速度
    std::vector<float> torques = motor_torques_;       // 每个电机的独立力矩
    std::vector<double> accels = motor_accels_;        // 每个电机的独立加速度
    std::vector<double> decels = motor_decels_;        // 每个电机的独立减速度

    bool success = eyou_can_->setProfilePositionMulti(
      ids, positions, speeds, torques, accels, decels);

    if (!success)
    {
      RCLCPP_WARN(rclcpp::get_logger("EyouSystem"), "发送位置命令失败");
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("EyouSystem"), "写入命令失败：%s", e.what());
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace eyou_test_ros2_control

// 注册硬件插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  eyou_test_ros2_control::EyouSystem,
  hardware_interface::SystemInterface)
