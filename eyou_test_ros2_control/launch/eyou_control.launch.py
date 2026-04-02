import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 获取功能包路径
    pkg_path = get_package_share_directory('eyou_test_ros2_control')
    
    # ========== 关键修改：直接指定纯URDF文件路径 ==========
    urdf_path = os.path.join(pkg_path, 'urdf', 'eyou_test.urdf')
    
    # 控制器配置文件路径
    config_path = os.path.join(pkg_path, 'config', 'eyou_controllers.yaml')

    # ========== 关键修改：直接读取URDF文件内容 ==========
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 机器人状态发布节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 控制器管理器节点
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, config_path],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # 加载关节状态广播器
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 加载位置控制器
    load_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_position_controller
    ])
