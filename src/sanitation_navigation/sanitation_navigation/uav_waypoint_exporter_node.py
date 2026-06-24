"""Export local ROS paths as GPS waypoint files for UAV missions."""

import csv
import json
import math
import os
from pathlib import Path

import rclpy
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


METERS_PER_DEGREE = 111194.9


def _local_to_latlon(x, y, origin_lat, origin_lon):
    cos_phi = math.cos(math.radians(origin_lat))
    if abs(cos_phi) < 1e-9:
        raise ValueError("origin latitude is too close to the poles")
    lon = origin_lon + x / (METERS_PER_DEGREE * cos_phi)
    lat = origin_lat + y / METERS_PER_DEGREE
    return lat, lon


def _find_project_root():
    env_root = os.environ.get("ROBOTDOG_ROOT")
    if env_root:
        return Path(env_root)
    p = Path(__file__).resolve()
    for parent in [p] + list(p.parents):
        if (parent / "data").is_dir() and (parent / "src").is_dir():
            return parent
    return Path.cwd()


class UavWaypointExporterNode(Node):
    """Convert /global_plan ENU path points to WGS-84 waypoint files.

    The local ENU origin can be locked from the first RTK fix, which is the
    intended mode for DJI Matrice 4E field tests. The generated CSV/JSON files
    are deliberately plain so they can be adapted to DJI Pilot/MSDK formats once
    the senior student's app interface is known.
    """

    def __init__(self):
        super().__init__("uav_waypoint_exporter_node")

        project_root = _find_project_root()
        default_output = str(project_root / "data" / "output")

        self.declare_parameter("gps_topic", "/fix")
        self.declare_parameter("plan_topic", "/global_plan")
        self.declare_parameter("waypoint_json_topic", "/uav/gps_waypoints_json")
        self.declare_parameter("origin_mode", "first_fix")  # "first_fix" or "param"
        self.declare_parameter("origin_lat", 30.747903)
        self.declare_parameter("origin_lon", 103.925269)
        self.declare_parameter("altitude_m", 30.0)
        self.declare_parameter("altitude_mode", "relative_to_takeoff")
        self.declare_parameter("speed_mps", 5.0)
        self.declare_parameter("min_spacing_m", 2.0)
        self.declare_parameter("min_waypoints", 2)
        self.declare_parameter("max_waypoints", 65535)
        self.declare_parameter("output_dir", default_output)
        self.declare_parameter("output_prefix", "uav_waypoints")
        self.declare_parameter("mission_name", "uav_area_mission")
        self.declare_parameter("coordinate_frame", "WGS84")
        self.declare_parameter("protocol", "DJI_WAYPOINT_3_0")
        self.declare_parameter("aircraft_model", "DJI Matrice 4E")

        self._gps_topic = self.get_parameter("gps_topic").value
        self._plan_topic = self.get_parameter("plan_topic").value
        self._json_topic = self.get_parameter("waypoint_json_topic").value
        self._origin_mode = self.get_parameter("origin_mode").value
        self._origin_lat = float(self.get_parameter("origin_lat").value)
        self._origin_lon = float(self.get_parameter("origin_lon").value)
        self._altitude_m = float(self.get_parameter("altitude_m").value)
        self._altitude_mode = self.get_parameter("altitude_mode").value
        self._speed_mps = float(self.get_parameter("speed_mps").value)
        self._min_spacing_m = float(self.get_parameter("min_spacing_m").value)
        self._min_waypoints = int(self.get_parameter("min_waypoints").value)
        self._max_waypoints = int(self.get_parameter("max_waypoints").value)
        self._output_dir = Path(str(self.get_parameter("output_dir").value))
        self._output_prefix = self.get_parameter("output_prefix").value
        self._mission_name = self.get_parameter("mission_name").value
        self._coordinate_frame = self.get_parameter("coordinate_frame").value
        self._protocol = self.get_parameter("protocol").value
        self._aircraft_model = self.get_parameter("aircraft_model").value

        if self._origin_mode not in ("first_fix", "param"):
            raise ValueError("origin_mode must be 'first_fix' or 'param'")
        if self._altitude_mode not in ("relative_to_takeoff", "absolute_amsl"):
            raise ValueError(
                "altitude_mode must be 'relative_to_takeoff' or 'absolute_amsl'"
            )
        if self._coordinate_frame not in ("WGS84", "CGCS2000"):
            raise ValueError("coordinate_frame must be 'WGS84' or 'CGCS2000'")
        if self._altitude_m <= 0.0:
            raise ValueError("altitude_m must be positive")
        if self._speed_mps <= 0.0:
            raise ValueError("speed_mps must be positive")
        if self._min_spacing_m < 0.0:
            raise ValueError("min_spacing_m must be non-negative")
        if self._min_waypoints < 2:
            raise ValueError("min_waypoints must be at least 2")
        if self._max_waypoints < self._min_waypoints:
            raise ValueError("max_waypoints must be >= min_waypoints")

        self._origin_locked = self._origin_mode == "param"
        self._pending_path = None

        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._json_pub = self.create_publisher(String, self._json_topic, 10)
        self._fix_sub = self.create_subscription(
            NavSatFix, self._gps_topic, self._fix_callback, 10
        )
        self._plan_sub = self.create_subscription(
            NavPath, self._plan_topic, self._path_callback, 10
        )

        if self._origin_locked:
            self.get_logger().info(
                f"UAV exporter using param origin: "
                f"({self._origin_lat:.8f}, {self._origin_lon:.8f})"
            )
        else:
            self.get_logger().info(
                f"UAV exporter waiting for first RTK fix on {self._gps_topic}"
            )

    def _fix_callback(self, msg):
        if self._origin_locked:
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            self.get_logger().warn("Ignoring invalid RTK fix")
            return

        self._origin_lat = msg.latitude
        self._origin_lon = msg.longitude
        self._origin_locked = True
        self.get_logger().info(
            f"UAV ENU origin locked from first RTK fix: "
            f"({self._origin_lat:.8f}, {self._origin_lon:.8f})"
        )

        if self._pending_path is not None:
            pending = self._pending_path
            self._pending_path = None
            self._export_path(pending)

    def _path_callback(self, msg):
        if not msg.poses:
            self.get_logger().warn("Ignoring empty /global_plan")
            return
        if not self._origin_locked:
            self._pending_path = msg
            self.get_logger().warn(
                "Received /global_plan before RTK origin; queued until first fix"
            )
            return
        self._export_path(msg)

    def _export_path(self, msg):
        points = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        points = self._downsample(points)
        if len(points) < self._min_waypoints:
            self.get_logger().warn(
                f"Path has {len(points)} waypoint(s), but {self._protocol} "
                f"requires at least {self._min_waypoints}; export skipped"
            )
            return

        waypoints = []
        for idx, (x, y) in enumerate(points):
            lat, lon = _local_to_latlon(x, y, self._origin_lat, self._origin_lon)
            waypoints.append(
                {
                    "index": idx,
                    "latitude": lat,
                    "longitude": lon,
                    "altitude_m": self._altitude_m,
                    "altitude_mode": self._altitude_mode,
                    "speed_mps": self._speed_mps,
                    "coordinate_frame": self._coordinate_frame,
                }
            )

        payload = {
            "mission_name": self._mission_name,
            "protocol": self._protocol,
            "aircraft_model": self._aircraft_model,
            "coordinate_frame": self._coordinate_frame,
            "altitude_mode": self._altitude_mode,
            "waypoint_limits": {
                "min": self._min_waypoints,
                "max": self._max_waypoints,
            },
            "origin": {
                "mode": self._origin_mode,
                "latitude": self._origin_lat,
                "longitude": self._origin_lon,
            },
            "source_topic": self._plan_topic,
            "source_frame": msg.header.frame_id,
            "waypoint_count": len(waypoints),
            "waypoints": waypoints,
        }

        json_path = self._output_dir / f"{self._output_prefix}.json"
        csv_path = self._output_dir / f"{self._output_prefix}.csv"
        json_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self._write_csv(csv_path, waypoints)

        out_msg = String()
        out_msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self._json_pub.publish(out_msg)

        self.get_logger().info(
            f"Exported {len(waypoints)} UAV GPS waypoints: "
            f"{csv_path} and {json_path}"
        )

    def _downsample(self, points):
        if len(points) <= 2:
            return points

        filtered = [points[0]]
        last_x, last_y = points[0]
        for x, y in points[1:-1]:
            if math.hypot(x - last_x, y - last_y) >= self._min_spacing_m:
                filtered.append((x, y))
                last_x, last_y = x, y
        if filtered[-1] != points[-1]:
            filtered.append(points[-1])

        if self._max_waypoints >= self._min_waypoints and len(filtered) > self._max_waypoints:
            max_count = self._max_waypoints
            sampled = []
            for i in range(max_count):
                src_idx = round(i * (len(filtered) - 1) / (max_count - 1))
                sampled.append(filtered[src_idx])
            filtered = sampled
        return filtered

    @staticmethod
    def _write_csv(path, waypoints):
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "index",
                    "latitude",
                    "longitude",
                    "altitude_m",
                    "altitude_mode",
                    "speed_mps",
                    "coordinate_frame",
                ],
            )
            writer.writeheader()
            writer.writerows(waypoints)


def main(args=None):
    rclpy.init(args=args)
    node = UavWaypointExporterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
