#!/usr/bin/env python3
import os

from typing import List

import xacro
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetParametersFromFile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration


def launch_setup(context: LaunchContext) -> List[GroupAction]:
    field_config = LaunchConfiguration("field_config")
    robot_config = LaunchConfiguration("robot_config")

    # LaunchConfigurationの中身を取得
    agent_prefix = LaunchConfiguration("agent_prefix", default="agent").perform(context)
    agent_id = LaunchConfiguration("agent_id").perform(context)
    agent_name = agent_prefix + agent_id

    # rviz上にロボットの3Dモデルを表示するための処理
    pkg_robot_model = get_package_share_directory("robot_model")
    xacro_file_path = os.path.join(pkg_robot_model, "urdf", "karugamot.urdf.xacro")
    assert os.path.exists(xacro_file_path)

    # xacro:argを用いてxacroファイル変数を渡すことができる
    # 複数台の場合はそれぞれ固有のrobot_idを付与する
    doc = xacro.process_file(xacro_file_path, mappings={"robot_id": str(agent_id), "robot_frame": "base"})
    robot_desc = doc.toxml()

    # GroupActionによりnamespaceやparameterを一括して与える
    agent_group = GroupAction(
        actions=[
            PushRosNamespace(agent_name),
            SetParameter(name="agent_id", value=agent_id),
            SetParametersFromFile(field_config),
            SetParametersFromFile(robot_config),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_desc}],
            ),
            Node(
                package="robot_model",
                executable="kinematic_agent",
            ),
            Node(
                package="robot_model",
                executable="posest2posevel",
            ),
            Node(
                package="los_controller",
                executable="joy_controller",
            ),
            Node(
                package="field_manager",
                executable="sensing_region_calculator",
            ),
            Node(
                package="field_manager",
                executable="sensing_region_marker_visualizer",
            ),
            # Node(
            #     package="field_manager",
            #     executable="sensing_region_pointcloud_visualizer",
            # ),
        ],
    )
    return [agent_group]


def generate_launch_description() -> LaunchDescription:
    DeclareLaunchArgument("field_config")
    DeclareLaunchArgument("agent_prefix")
    DeclareLaunchArgument("agent_id")

    # Create the launch description and populate
    ld = LaunchDescription()

    # LaunchConfigurationの値を取得するため，OpaqueFunctionで外部実行
    # ref: https://answers.ros.org/question/340705/access-launch-argument-in-launchfile-ros2/
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
