"""题目一：robot_state_publisher + RViz2 + q1_drive 一体化启动。

用法：
  ros2 launch bajie_tasks q1_rviz.launch.py
  ros2 launch bajie_tasks q1_rviz.launch.py \
      input_file:=/workspace/inputs/q1_demo_input.jsonl \
      output_file:=/workspace/answers/q1_demo_output.jsonl

注意：这里不启动 joint_state_publisher_gui，/joint_states 由 q1_drive 独占发布。
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_param_builder import load_xacro


def generate_launch_description():
    robot_share = get_package_share_directory("bajie_robot")
    # load_xacro 需要 Path，不能传 str
    xacro_path = Path(robot_share) / "urdf" / "bajie_robot.urdf.xacro"
    robot_description = load_xacro(file_path=xacro_path)

    input_file = LaunchConfiguration("input_file")
    output_file = LaunchConfiguration("output_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "input_file", default_value="/workspace/inputs/q1_demo_input.jsonl"),
        DeclareLaunchArgument(
            "output_file", default_value="/workspace/answers/q1_demo_output.jsonl"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        ),
        Node(
            package="bajie_tasks",
            executable="q1_drive",
            name="q1_drive",
            output="screen",
            parameters=[{"input_file": input_file, "output_file": output_file}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", os.path.join(robot_share, "rviz", "display.rviz")],
        ),
    ])
