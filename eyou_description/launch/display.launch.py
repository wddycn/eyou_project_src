from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 获取包路径
    pkg_share = get_package_share_directory('eyou_description')

    # URDF 文件
    urdf_file = os.path.join(pkg_share, 'urdf', 'eyou.urdf')

    # RViz 配置
    rviz_config = os.path.join(pkg_share, 'urdf.rviz')

    # 声明可选参数 model（与 ROS1 保持一致）
    declare_model_arg = DeclareLaunchArgument(
        name='model',
        default_value='eyou',
        description='Robot model name'
    )

    # robot_state_publisher 需要 robot_description 参数
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    # joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        declare_model_arg,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
