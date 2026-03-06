#!/usr/bin/env python3

from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Header

# Z offset to display trajectory above the robot model
TRAJECTORY_Z_OFFSET = 0.25


class Footprinter(Node):
    """Track and visualize agent trajectories"""

    def __init__(self) -> None:
        super().__init__("footprinter")

        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.world_frame = str(self.get_parameter("world_frame").value)

        self.poses: List[PoseStamped] = []

        # pub
        self.trajectory_pub = self.create_publisher(Path, "robot_trajectory", 10)

        # sub
        self.create_subscription(PoseStamped, "pose", self.posest_callback, 10)

    def posest_callback(self, msg: PoseStamped) -> None:
        msg.pose.position.z += TRAJECTORY_Z_OFFSET
        self.poses.append(msg)
        trajectory = Path(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame),
            poses=self.poses,
        )
        self.trajectory_pub.publish(trajectory)


def main() -> None:
    rclpy.init()
    node = Footprinter()

    try:
        rclpy.spin(node)
    except Exception:
        node.get_logger().error("Unexpected error", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
