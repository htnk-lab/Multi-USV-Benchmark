#!/usr/bin/env python3

import os
from typing import List, Union

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParameter, SetParametersFromFile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(
    context: LaunchContext,
) -> List[Union[Node, GroupAction, LaunchDescription]]:
    agent_num = int(LaunchConfiguration("num", default=2).perform(context))
    assert agent_num in range(1, 5), f"invalid agent_num: {agent_num}"

    pkg_benchmark_main = get_package_share_directory("benchmark_main")
    pkg_robot_model = get_package_share_directory("robot_model")
    pkg_field_manager = get_package_share_directory("field_manager")
    pkg_los_controller = get_package_share_directory("los_controller")

    rviz_config = os.path.join(pkg_field_manager, "rviz", "field.rviz")
    assert os.path.exists(rviz_config)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    field_config = os.path.join(pkg_field_manager, "config", "field.params.yaml")
    robot_config = os.path.join(pkg_robot_model, "config", "robot.params.yaml")
    los_config = os.path.join(pkg_los_controller, "config", "los.params.yaml")

    # group actionでまとめることでconfigを共通で与える
    central_group = GroupAction(
        actions=[
            SetParametersFromFile(field_config),
            SetParametersFromFile(robot_config),
            SetParametersFromFile(los_config),
            SetParameter(name="agent_num", value=agent_num),
            Node(
                package="field_manager",
                executable="pose_collector",
                output="screen",
            ),
            Node(
                package="field_manager",
                executable="central",
            ),
            # Node(package="field_manager", executable="phi_marker_visualizer"),
            Node(package="field_manager", executable="phi_pointcloud_visualizer"),
            Node(package="joy", executable="joy_node"),
            Node(
                package="los_controller",
                executable="convex_polygon_creator",
            ),
            Node(
                package="field_manager",
                executable="pool_visualizer",
                output="screen",
            ),
        ]
    )

    agent_launch_list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_benchmark_main, "launch", "agent.launch.py")]
            ),
            launch_arguments={
                "field_config": field_config,
                "robot_config": robot_config,
                "los_config": los_config,
                "agent_id": str(agent_id),
                "agent_num": str(agent_num),
            }.items(),
        )
        for agent_id in range(agent_num)
    ]
    # nodeの起動順に起因するagentのジャンプを防ぐため，central系を後に
    return agent_launch_list + [rviz_node, central_group]


def generate_launch_description() -> LaunchDescription:
    DeclareLaunchArgument("num", description="< 5")

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
