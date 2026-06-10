#!/home/ubuntu/bl/miniconda3/envs/robotdog/bin/python
"""Profile the default DA3 PointCloud2 local-planning ROS2 chain."""

import argparse
import signal
import statistics
import time
from collections import defaultdict

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


STAGES = ("image", "camera_info", "depth", "raw_cloud", "filtered_cloud", "costmap", "path")


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class PipelineProfiler(Node):
    def __init__(self, max_frames, timeout):
        super().__init__("pointcloud_pipeline_profiler")
        self.max_frames, self.timeout = max_frames, timeout
        self.frames, self.samples = {}, defaultdict(list)
        self.completed, self.finished = 0, False
        self.started = time.perf_counter()
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        self.create_subscription(Image, "/camera/image_raw", lambda m: self.mark("image", m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera_info", lambda m: self.mark("camera_info", m), qos_profile_sensor_data)
        self.create_subscription(Image, "/depth_anything_v3/output/depth_image", lambda m: self.mark("depth", m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/depth_anything_v3/output/point_cloud", lambda m: self.mark("raw_cloud", m), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/depth_anything/points_filtered", lambda m: self.mark("filtered_cloud", m), qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/local_occupancy_grid", lambda m: self.mark("costmap", m), 10)
        self.create_subscription(Path, "/local_path", lambda m: self.mark("path", m), 10)
        self.create_timer(0.5, self.housekeeping)

    def mark(self, stage, msg):
        key = stamp_ns(msg)
        if key <= 0:
            return
        frame = self.frames.setdefault(key, {})
        frame.setdefault(stage, time.perf_counter_ns())
        if stage == "path" and all(name in frame for name in STAGES):
            start = max(frame["image"], frame["camera_info"])
            values = {
                "da3": (frame["depth"] - start) / 1e6,
                "pointcloud_publish": (frame["raw_cloud"] - frame["depth"]) / 1e6,
                "ground_filter": (frame["filtered_cloud"] - frame["raw_cloud"]) / 1e6,
                "local_costmap_builder": (frame["costmap"] - frame["filtered_cloud"]) / 1e6,
                "local_astar_planner": (frame["path"] - frame["costmap"]) / 1e6,
                "total_image_to_path": (frame["path"] - frame["image"]) / 1e6,
            }
            for name, value in values.items():
                if value >= 0.0:
                    self.samples[name].append(value)
            self.completed += 1
            self.frames.pop(key, None)
            if self.max_frames > 0 and self.completed >= self.max_frames:
                self.finished = True

    def housekeeping(self):
        cutoff = time.perf_counter_ns() - 30_000_000_000
        self.frames = {key: frame for key, frame in self.frames.items() if max(frame.values()) >= cutoff}
        if self.timeout > 0 and time.perf_counter() - self.started >= self.timeout:
            self.finished = True

    def report(self):
        print(f"frames={self.completed}")
        print("stage                         mean_ms median_ms   p95_ms      FPS")
        for name in (
            "da3", "pointcloud_publish", "ground_filter", "local_costmap_builder",
            "local_astar_planner", "total_image_to_path",
        ):
            values = self.samples.get(name, [])
            if values:
                mean = statistics.fmean(values)
                print(f"{name:28s} {mean:8.3f} {statistics.median(values):9.3f} {np.percentile(values, 95):8.3f} {1000.0 / mean:8.2f}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=180.0)
    args = parser.parse_args()
    rclpy.init()
    node = PipelineProfiler(args.frames, args.timeout)
    signal.signal(signal.SIGINT, lambda *_: setattr(node, "finished", True))
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, "finished", True))
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
