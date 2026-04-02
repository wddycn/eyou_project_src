#ifndef EYOU_ROS2_CONTROL__EYOU_SYSTEM_HPP_
#define EYOU_ROS2_CONTROL__EYOU_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

// 引入你的第三方库
// 确保 eyou.h 在包含路径中，或者修改为实际路径
#include "eyou.h" 

namespace eyou_ros2_control
{

class EyouSystem : public hardware_interface::SystemInterface
{
public:
  EyouSystem();
  virtual ~EyouSystem();

  // 初始化硬件参数 (从 URDF 解析)
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // 导出状态接口
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 导出命令接口
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 硬件激活
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // 硬件停用
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // 读取硬件状态
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 写入硬件命令
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // CAN 实例
  std::unique_ptr<MotorCAN> eyou_can_;
  
  // 电机 ID 列表
  std::vector<uint8_t> motor_ids_;

  // 状态数据: [pos1, vel1, eff1, pos2, vel2, eff2, ...]
  std::vector<double> hw_states_;
  
  // 命令数据: [pos_cmd1, vel_cmd1(预留), eff_cmd1(预留), ...]
  // 目前主要使用 position 命令
  std::vector<double> hw_commands_;

  // 配置参数
  std::string can_interface_name_;
  double max_speed_;
  double max_accel_;
  double max_torque_;
  std::vector<double> joint_max_speeds_;
  std::vector<double> joint_max_accels_;
  std::vector<double> joint_max_torques_;
};

}  // namespace eyou_ros2_control

#endif  // EYOU_ROS2_CONTROL__EYOU_SYSTEM_HPP_