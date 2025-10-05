#!/usr/bin/env python3
import traceback

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Header


class Footprinter(Node):
    def __init__(self) -> None:
        super().__init__("footprinter")

        # declare parameter
        self.declare_parameters(
            namespace="",
            parameters=[
                ("world_frame", "world"),
            ],
        )
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value

        # initialization
        self.poses = []

        # pub
        self.trajectory_pub = self.create_publisher(Path, "robot_trajectory", 10)

        # sub
        self.create_subscription(PoseStamped, "pose", self.posest_callback, 10)

    def posest_callback(self, msg: PoseStamped) -> None:
        curr_msg = msg
        curr_msg.pose.position.z = curr_msg.pose.position.z + 0.25
        self.poses = self.poses + [curr_msg]
        self.trajectory = Path(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame), poses=self.poses
        )
        self.trajectory_pub.publish(self.trajectory)


def main() -> None:
    rclpy.init()
    footprinter = Footprinter()

    try:
        rclpy.spin(footprinter)
    except:
        footprinter.get_logger().error(traceback.format_exc())
    finally:
        footprinter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
