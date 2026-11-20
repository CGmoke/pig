import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import launch_ros.actions as ros_actions
import launch.actions as actions
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
from launch_param_builder import ParameterBuilder, load_yaml, load_xacro
# import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 获取URDF文件的路径
    package_name = 'bajie_robot'

    pkg_path=Path(get_package_share_directory('bajie_robot'))
    urdf_path=pkg_path/'urdf/bajie_robot.urdf.xacro'

    robot_description = load_xacro(
        file_path=urdf_path,
        mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )}
    )

    DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # 创建URDF节点
    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, 
                    {'robot_description': robot_description}
                    ]
    )

    joint_state_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='screen',
    )

    rviz_file = os.path.join(get_package_share_directory(package_name), 'rviz/display.rviz')

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file]
        )

    return LaunchDescription([robot_state_node, 
                            #   joint_state_node, 
                              joint_state_gui_node,
                              rviz_node
                              ])