#!/usr/bin/env python3

import os
from typing import List

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetParametersFromFile


def launch_setup(context: LaunchContext) -> List[GroupAction]:
    central_config = LaunchConfiguration("central_config")
    robot_config = LaunchConfiguration("robot_config")
    controller_config = LaunchConfiguration("controller_config")

    agent_prefix = LaunchConfiguration("agent_prefix", default="agent").perform(context)
    agent_id = LaunchConfiguration("agent_id").perform(context)
    agent_num = LaunchConfiguration("agent_num").perform(context)
    agent_name = agent_prefix + agent_id

    pkg_robot_model = get_package_share_directory("robot_model")
    xacro_file_path = os.path.join(pkg_robot_model, "urdf", "robot.urdf.xacro")
    assert os.path.exists(xacro_file_path)

    doc = xacro.process_file(xacro_file_path, mappings={"robot_id": str(agent_id), "robot_frame": "base"})
    robot_desc = doc.toxml()

    central_agent_nodes = GroupAction(
        actions=[
            PushRosNamespace(agent_name),
            SetParameter(name="agent_id", value=agent_id),
            SetParameter(name="agent_num", value=agent_num),
            SetParametersFromFile(central_config),
            Node(
                package="field_manager",
                executable="sensing_region_calculator",
            ),
            Node(
                package="field_manager",
                executable="sensing_region_marker_visualizer",
            ),
        ]
    )

    robot_nodes = GroupAction(
        actions=[
            PushRosNamespace(agent_name),
            SetParameter(name="agent_id", value=agent_id),
            SetParameter(name="agent_num", value=agent_num),
            SetParametersFromFile(central_config),
            SetParametersFromFile(robot_config),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_desc}],
            ),
            Node(
                package="robot_model",
                executable="ideal_agent",
            ),
            Node(
                package="robot_model",
                executable="posest2posevel",
            ),
            Node(
                package="robot_model",
                executable="footprinter",
            ),
        ]
    )

    controller_nodes = GroupAction(
        actions=[
            PushRosNamespace(agent_name),
            SetParameter(name="agent_id", value=agent_id),
            SetParameter(name="agent_num", value=agent_num),
            SetParametersFromFile(central_config),
            SetParametersFromFile(controller_config),
            Node(
                package="controller",
                executable="waypoints_generator",
            ),
            Node(
                package="controller",
                executable="los",
            ),
            Node(
                package="controller",
                executable="angle_fbcontroller",
            ),
        ],
    )
    return [central_agent_nodes, robot_nodes, controller_nodes]


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("central_config"))
    ld.add_action(DeclareLaunchArgument("robot_config"))
    ld.add_action(DeclareLaunchArgument("controller_config"))
    ld.add_action(DeclareLaunchArgument("agent_prefix", default_value="agent"))
    ld.add_action(DeclareLaunchArgument("agent_id"))
    ld.add_action(DeclareLaunchArgument("agent_num"))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
