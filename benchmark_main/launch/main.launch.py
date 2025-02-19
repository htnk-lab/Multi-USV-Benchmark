#!/usr/bin/env python3

import os
from typing import List, Union

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, SetParametersFromFile

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
    agent_num = int(LaunchConfiguration("num", default=3).perform(context))
    assert agent_num in range(1, 6), f"invalid agent_num: {agent_num}"

    pkg_field_manager = get_package_share_directory("field_manager")
    rviz_config = os.path.join(pkg_field_manager, "rviz", "field.rviz")
    assert os.path.exists(rviz_config)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    field_config = os.path.join(pkg_field_manager, "config", "field.params.yaml")

    # group actionでまとめることでconfigを共通で与える
    central_group = GroupAction(
        actions=[
            SetParametersFromFile(field_config),
            Node(
                package="field_manager",
                executable="pose_collector",
                parameters=[{"agent_num": agent_num}],
                output="screen",
            ),
            Node(
                package="field_manager",
                executable="central",
            ),
            # Node(package="field_manager", executable="phi_marker_visualizer"),
            Node(package="field_manager", executable="phi_pointcloud_visualizer"),
        ]
    )

    agent_launch_list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(pkg_field_manager, "launch", "agent.launch.py")]
            ),
            launch_arguments={
                "field_config": field_config,
                "agent_id": str(agent_id),
            }.items(),
        )
        for agent_id in range(agent_num)
    ]
    # nodeの起動順に起因するagentのジャンプを防ぐため，central系を後に
    return agent_launch_list + [rviz_node, central_group]


def generate_launch_description() -> LaunchDescription:
    DeclareLaunchArgument("num", description="< 3")

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
