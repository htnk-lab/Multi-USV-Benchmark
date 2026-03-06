#!/usr/bin/env python

from dataclasses import dataclass, field
from functools import partial
from typing import List

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Header


@dataclass
class Data:
    curr_pose: Pose = field(default_factory=Pose)
    is_ready: bool = False


class PoseCollector(Node):
    """Collect all agents' current poses and publish them as a PoseArray"""

    def __init__(self) -> None:
        super().__init__("pose_collector")

        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter("agent_num", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "agent_prefix", "agent", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "timer_period", 0.1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        self.world_frame = str(self.get_parameter("world_frame").value)
        agent_num = int(self.get_parameter("agent_num").value)
        agent_prefix = str(self.get_parameter("agent_prefix").value)

        self.data_list: List[Data] = [Data() for _ in range(agent_num)]
        self.is_ready = False

        timer_period = float(self.get_parameter("timer_period").value)

        # pub
        self.curr_pose_array_pub = self.create_publisher(PoseArray, "curr_pose_array", 10)

        # sub: create per-agent subscriptions
        for agent_id in range(agent_num):
            agent_name = agent_prefix + str(agent_id)
            topic_name = agent_name + "/curr_pose"
            self.create_subscription(Pose, topic_name, partial(self.curr_pose_callback, agent_id=agent_id), 10)

        # timer
        self.create_timer(timer_period, self.timer_callback)

    def curr_pose_callback(self, msg: Pose, agent_id: int) -> None:
        assert isinstance(agent_id, int)
        self.data_list[agent_id] = Data(curr_pose=msg, is_ready=True)

    def timer_callback(self) -> None:
        if self.is_ready:
            # publish poses ordered by agent_id
            curr_pose_array = [data.curr_pose for data in self.data_list]
            self.curr_pose_array_pub.publish(
                PoseArray(
                    header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame),
                    poses=curr_pose_array,
                )
            )
        else:
            if all(data.is_ready for data in self.data_list):
                self.get_logger().info("pose_collector is ready")
                self.is_ready = True


def main() -> None:
    rclpy.init()
    pose_collector = PoseCollector()

    try:
        rclpy.spin(pose_collector)
    except Exception:
        pose_collector.get_logger().error("Unexpected error", exc_info=True)
    finally:
        pose_collector.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
