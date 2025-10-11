import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rclpy.node import Node


class ConvexPolygonCreator(Node):
    def __init__(self):
        super().__init__("convex_polygon_creator")
        self.declare_parameter("agent_num", 2)
        self.declare_parameter("x_limit", [0.0, 10.0])
        self.declare_parameter("y_limit", [0.0, 10.0])
        self.declare_parameter("agent_prefix", "agent")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("timer_period", 1.0)

        self.agent_num = self.get_parameter("agent_num").value
        self.x_limit = self.get_parameter("x_limit").value
        self.y_limit = self.get_parameter("y_limit").value
        self.agent_prefix = self.get_parameter("agent_prefix").value
        self.world_frame = self.get_parameter("world_frame").value
        timer_period = self.get_parameter("timer_period").value

        self.polygon_pub_list = [
            self.create_publisher(PolygonStamped, f"{self.agent_prefix}{i}/field_range", 10)
            for i in range(self.agent_num)
        ]

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        for agent_id in range(self.agent_num):
            x_length = self.x_limit[1] - self.x_limit[0]
            part_x = [
                self.x_limit[0] + x_length * agent_id / self.agent_num,
                self.x_limit[0] + x_length * (agent_id + 1) / self.agent_num,
            ]
            points = np.array(
                [
                    [part_x[0], self.y_limit[0], 0.0],
                    [part_x[0], self.y_limit[1], 0.0],
                    [part_x[1], self.y_limit[1], 0.0],
                    [part_x[1], self.y_limit[0], 0.0],
                ]
            )
            if points.shape[0] > 2:
                if not self.is_convex(points):
                    points = np.zeros((0, 3))
            self.publish_polygon(points, agent_id)

    def is_convex(self, points: np.ndarray) -> bool:
        if points.shape[0] < 3:
            return True
        count = 0
        for i in range(points.shape[0]):
            a = points[i]
            b = points[(i + 1) % points.shape[0]]
            c = points[(i + 2) % points.shape[0]]
            count += 1 if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) < 0 else -1
        if count == points.shape[0] or count == -points.shape[0]:
            return True
        return False

    def publish_polygon(self, points: np.ndarray, agent_id: int) -> None:
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = self.world_frame
        for point in points:
            p = Point32()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            polygon_msg.polygon.points.append(p)
        self.polygon_pub_list[agent_id].publish(polygon_msg)

    def spin(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    polygon_creator = ConvexPolygonCreator()
    polygon_creator.spin()
    polygon_creator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
