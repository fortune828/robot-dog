"""perception.launch.py — Phase 1 求生本能：单目深度 → 3D 点云 → 地面过滤

Launches:
  depth_to_cloud_node — Depth Anything V2 → pinhole back-projection → /camera/depth_points
  ground_filter_node — Z-axis pass-through filter → /depth_anything/points_filtered
                      + static TF base_link → camera_link

所有参数集中在 default_params.yaml 中管理。仅 video_path/stride/device 支持命令行覆盖。

用法:
  ros2 launch sanitation_bringup perception.launch.py
  ros2 launch sanitation_bringup perception.launch.py stride:=8 device:=cpu
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 仅暴露需要命令行覆盖的参数
    video_path_arg = DeclareLaunchArgument(
        "video_path", default_value="",
        description="MP4 video file (auto → data/test_video.mp4)",
    )
    stride_arg = DeclareLaunchArgument(
        "stride", default_value="4",
        description="Point cloud downsampling step",
    )
    device_arg = DeclareLaunchArgument(
        "device", default_value="auto",
        description="'cuda', 'cpu', or 'auto'",
    )

    video_path = LaunchConfiguration("video_path")
    stride = LaunchConfiguration("stride")
    device = LaunchConfiguration("device")

    config_dir = os.path.join(
        get_package_share_directory("sanitation_bringup"), "config"
    )
    default_config = os.path.join(config_dir, "default_params.yaml")

    depth_to_cloud_node = Node(
        package="sanitation_perception",
        executable="depth_to_cloud_node",
        name="depth_to_cloud_node",
        output="screen",
        parameters=[default_config, {
            "video_path": video_path,
            "stride": stride,
            "device": device,
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    ground_filter_node = Node(
        package="sanitation_perception",
        executable="ground_filter_node",
        name="ground_filter_node",
        output="screen",
        parameters=[default_config],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription([
        video_path_arg, stride_arg, device_arg,
        LogInfo(msg="=== Phase 1: monocular depth → 3D point cloud → ground filter ==="),
        LogInfo(msg="depth_to_cloud  → /camera/depth_points (camera_link)"),
        LogInfo(msg="ground_filter   → /depth_anything/points_filtered (camera_link)"),
        LogInfo(msg="static TF: base_link → camera_link"),
        depth_to_cloud_node,
        ground_filter_node,
    ])
