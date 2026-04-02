import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('eyou_ros2_control')

    urdf_file = os.path.join(pkg_path, 'urdf', 'eyou_arm.urdf')
    controllers_file = os.path.join(pkg_path, 'config', 'eyou_controllers.yaml')

    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # 1. robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. ros2_control
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )

    # 3. joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    # ✅ 4. 改这里：用 position controller
    joint_group_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_group_position_controller',   
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        joint_group_position_controller_spawner,   
    ])