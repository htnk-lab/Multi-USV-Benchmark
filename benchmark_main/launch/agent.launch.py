#!/usr/bin/env python3

from typing import List

from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetParametersFromFile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration


def launch_setup(context: LaunchContext) -> List[GroupAction]:
    field_config = LaunchConfiguration("field_config")

    # LaunchConfigurationの中身を取得
    agent_prefix = LaunchConfiguration("agent_prefix", default="agent").perform(context)
    agent_id = LaunchConfiguration("agent_id").perform(context)
    agent_name = agent_prefix + agent_id

    # GroupActionによりnamespaceやparameterを一括して与える
    agent_group = GroupAction(
        actions=[
            PushRosNamespace(agent_name),
            SetParameter(name="agent_id", value=agent_id),
            SetParametersFromFile(field_config),
            Node(
                package="field_manager",
                executable="agent_body",
            ),
            Node(
                package="field_manager",
                executable="controller",
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
