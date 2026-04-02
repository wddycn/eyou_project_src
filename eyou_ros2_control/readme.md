# 控制命令
## 轨迹规划指令
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
  points:
    - positions: [0.5, -0.5, 0.5, -0.5, 0.5, 0.5]
      time_from_start: {sec: 2, nanosec: 0}
"
```
## 角度直接控制指令
```bash
ros2 topic pub /joint_group_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```