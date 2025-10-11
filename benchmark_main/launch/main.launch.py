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
    ExecuteProcess,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(
    context: LaunchContext,
) -> List[Union[Node, GroupAction, LaunchDescription]]:
    agent_num = int(LaunchConfiguration("num", default=2).perform(context))
    assert agent_num in range(1, 5), f"invalid agent_num: {agent_num}"
    bag_name = str(LaunchConfiguration("name", default="").perform(context))
    cwd = os.getcwd()
    bag_path = os.path.join(cwd, "src/Multi-USV-Benchmark/ros2bag2csv", bag_name)

    pkg_benchmark_main = get_package_share_directory("benchmark_main")
    pkg_robot_model = get_package_share_directory("robot_model")
    pkg_field_manager = get_package_share_directory("field_manager")
    pkg_controller = get_package_share_directory("controller")
    
    central_config = os.path.join(pkg_field_manager, "config", "central.params.yaml")
    robot_config = os.path.join(pkg_robot_model, "config", "robot.params.yaml")
    controller_config = os.path.join(pkg_controller, "config", "controller.params.yaml")
    rviz_config = os.path.join(pkg_field_manager, "rviz", "field.rviz")
    assert os.path.exists(rviz_config)
    
    visualization_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
    )

    logging_node = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_path, '-a'],
        output='screen'
    )

    # By using GroupAction, configs are provided commonly across nodes
    central_fields_nodes = GroupAction(
        actions=[
            SetParametersFromFile(central_config),
            SetParametersFromFile(robot_config),
            SetParametersFromFile(controller_config),
            SetParameter(name="agent_num", value=agent_num),
            Node(
                package="field_manager",
                executable="pose_collector",
                output="screen",
            ),
            Node(
                package="field_manager",
                executable="phi_update",
            ),
            Node(package="field_manager", executable="phi_pointcloud_visualizer"),
            Node(
                package="field_manager",
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
                "central_config": central_config,
                "robot_config": robot_config,
                "controller_config": controller_config,
                "agent_id": str(agent_id),
                "agent_num": str(agent_num),
            }.items(),
        )
        for agent_id in range(agent_num)
    ]
    # To prevent agent jumps due to node startup order, launch central nodes later
    return ([logging_node] if bag_name else []) + agent_launch_list + [visualization_node, central_fields_nodes]


def generate_launch_description() -> LaunchDescription:
    DeclareLaunchArgument("num", description="Number of agents")
    DeclareLaunchArgument("name", description="Name of the output rosbag")

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
