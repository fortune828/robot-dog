"""Publish a demo patrol polygon for desktop smoke tests."""

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class DemoPolygonNode(Node):
    """Publishes a simple closed patrol polygon to /clicked_point once."""

    def __init__(self):
        super().__init__("demo_polygon_node")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("publish_delay_sec", 2.0)
        self.declare_parameter(
            "points",
            [0.0, 0.0, 20.0, 0.0, 20.0, 12.0, 0.0, 12.0, 0.2, 0.2],
        )

        self._topic = self.get_parameter("clicked_point_topic").value
        self._world_frame = self.get_parameter("world_frame").value
        self._points = self._parse_points(self.get_parameter("points").value)
        delay = float(self.get_parameter("publish_delay_sec").value)
        self._pub = self.create_publisher(PointStamped, self._topic, 10)
        self._timer = self.create_timer(max(delay, 0.1), self._publish_once)
        self._published = False

    @staticmethod
    def _parse_points(raw):
        values = [float(v) for v in raw]
        points = []
        for i in range(0, len(values) - 1, 2):
            points.append((values[i], values[i + 1]))
        if len(points) < 4:
            raise ValueError("demo patrol polygon needs at least four points")
        return points

    def _publish_once(self):
        if self._published:
            return
        stamp = self.get_clock().now().to_msg()
        for x, y in self._points:
            msg = PointStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self._world_frame
            msg.point.x = x
            msg.point.y = y
            self._pub.publish(msg)
        self._published = True
        self.get_logger().info(
            f"Published demo patrol polygon with {len(self._points)} clicked points"
        )


def main(args=None):
    rclpy.init(args=args)
    node = DemoPolygonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
