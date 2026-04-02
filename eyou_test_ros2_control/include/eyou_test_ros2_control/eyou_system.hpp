#ifndef EYOU_TEST_ROS2_CONTROL__EYOU_SYSTEM_HPP_
#define EYOU_TEST_ROS2_CONTROL__EYOU_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

// ROS2 Control 核心头文件
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Eyou 电机驱动头文件（请确保路径正确）
#include "eyou.h"

namespace eyou_test_ros2_control
{
class EyouSystem : public hardware_interface::SystemInterface
{
public:
  // 移除多余分号
  RCLCPP_SHARED_PTR_DEFINITIONS(EyouSystem)

  // 初始化硬件接口
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // 获取状态接口（用于读取电机状态：位置、速度、力矩）
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 获取命令接口（用于发送控制命令：位置、速度、力矩）
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 启动硬件（使能电机）
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 停止硬件（失能电机）
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 读取电机状态（周期调用）
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 写入控制命令（周期调用）
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Eyou 电机驱动实例
  std::unique_ptr<MotorCAN> eyou_can_;
  
  // 电机配置
  std::vector<uint8_t> motor_ids_;  // 电机ID列表 [13, 10]
  size_t num_motors_;               // 电机数量
  
  // 控制模式（默认位置模式）
  enum class ControlMode { POSITION, VELOCITY, TORQUE };
  ControlMode control_mode_;

  // 状态数据（读取）：位置(rad)、速度(rad/s)、力矩(Nm)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_torques_;

  // 命令数据（写入）：位置(rad)、速度(rad/s)、力矩(Nm)
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_torque_;

  // CAN总线名称（从URDF配置读取）
  std::string can_interface_;

  // ========== 新增：每个电机的独立运动参数 ==========
  std::vector<double> motor_speeds_;    // 每个电机的目标速度(rad/s)
  std::vector<float> motor_torques_;    // 每个电机的目标力矩(Nm)
  std::vector<double> motor_accels_;    // 每个电机的加速度(rad/s²)
  std::vector<double> motor_decels_;    // 每个电机的减速度(rad/s²)
};

}  // namespace eyou_test_ros2_control

#endif  // EYOU_TEST_ROS2_CONTROL__EYOU_SYSTEM_HPP_
