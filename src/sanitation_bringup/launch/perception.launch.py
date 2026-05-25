"""perception.launch.py — Phase 1 求生本能：单目深度 → 3D 点云 → 地面过滤

Launches:
  depth_to_cloud_node — Depth Anything V2 → pinhole back-projection → /camera/depth_points (PointCloud2)
  ground_filter_node — Z-axis pass-through filter → /depth_anything/points_filtered (PointCloud2)
                       + static TF base_link → camera_link (补全 TF 树)

Usage:
  ros2 launch sanitation_bringup perception.launch.py
  ros2 launch sanitation_bringup perception.launch.py stride:=8 min_z:=0.1 max_z:=0.9
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ---- depth_to_cloud 参数 ----
    video_path_arg = DeclareLaunchArgument(
        "video_path", default_value="",
        description="MP4 video file (auto-resolves to data/test_video.mp4)",
    )
    encoder_arg = DeclareLaunchArgument(
        "encoder", default_value="vits",
        description="Encoder variant: vits, vitb, vitl",
    )
    weights_path_arg = DeclareLaunchArgument(
        "weights_path", default_value="",
        description="Path to .pth weights (auto-resolves to data/)",
    )
    depth_anything_home_arg = DeclareLaunchArgument(
        "depth_anything_home", default_value="",
        description="Path to Depth-Anything-V2 repo root",
    )
    device_arg = DeclareLaunchArgument(
        "device", default_value="auto",
        description="'cuda', 'cpu', or 'auto'",
    )
    inference_rate_arg = DeclareLaunchArgument(
        "inference_rate", default_value="10.0",
        description="Inference + publish rate (Hz)",
    )
    stride_arg = DeclareLaunchArgument(
        "stride", default_value="4",
        description="Spatial downsampling step (4 = ~19k pts, 8 = ~5k pts)",
    )
    min_depth_arg = DeclareLaunchArgument(
        "min_depth", default_value="0.1",
        description="Minimum depth (m)",
    )
    max_depth_arg = DeclareLaunchArgument(
        "max_depth", default_value="10.0",
        description="Maximum depth (m)",
    )

    # ---- ground_filter 参数 ----
    filter_min_z_arg = DeclareLaunchArgument(
        "min_z", default_value="0.15",
        description="Ground filter: Z below this is ground (stripped)",
    )
    filter_max_z_arg = DeclareLaunchArgument(
        "max_z", default_value="0.80",
        description="Ground filter: Z above this is sky/branches (stripped)",
    )
    camera_x_arg = DeclareLaunchArgument(
        "camera_x", default_value="0.15",
        description="Camera forward offset from base_link (m)",
    )
    camera_y_arg = DeclareLaunchArgument(
        "camera_y", default_value="0.0",
        description="Camera lateral offset from base_link (m)",
    )
    camera_z_arg = DeclareLaunchArgument(
        "camera_z", default_value="0.0",
        description="Camera vertical offset from base_link (m)",
    )

    video_path = LaunchConfiguration("video_path")
    encoder = LaunchConfiguration("encoder")
    weights_path = LaunchConfiguration("weights_path")
    depth_anything_home = LaunchConfiguration("depth_anything_home")
    device = LaunchConfiguration("device")
    inference_rate = LaunchConfiguration("inference_rate")
    stride = LaunchConfiguration("stride")
    min_depth = LaunchConfiguration("min_depth")
    max_depth = LaunchConfiguration("max_depth")
    min_z = LaunchConfiguration("min_z")
    max_z = LaunchConfiguration("max_z")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")

    config_dir = os.path.join(
        get_package_share_directory("sanitation_bringup"), "config"
    )
    default_config = os.path.join(config_dir, "default_params.yaml")

    depth_to_cloud_node = Node(
        package="sanitation_perception",
        executable="depth_to_cloud_node",
        name="depth_to_cloud_node",
        output="screen",
        parameters=[
            default_config,
            {
                "video_path": video_path,
                "encoder": encoder,
                "weights_path": weights_path,
                "depth_anything_home": depth_anything_home,
                "device": device,
                "inference_rate": inference_rate,
                "stride": stride,
                "min_depth": min_depth,
                "max_depth": max_depth,
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    ground_filter_node = Node(
        package="sanitation_perception",
        executable="ground_filter_node",
        name="ground_filter_node",
        output="screen",
        parameters=[
            default_config,
            {
                "min_z": min_z,
                "max_z": max_z,
                "camera_x": camera_x,
                "camera_y": camera_y,
                "camera_z": camera_z,
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            # depth_to_cloud args
            video_path_arg,
            encoder_arg,
            weights_path_arg,
            depth_anything_home_arg,
            device_arg,
            inference_rate_arg,
            stride_arg,
            min_depth_arg,
            max_depth_arg,
            # ground_filter args
            filter_min_z_arg,
            filter_max_z_arg,
            camera_x_arg,
            camera_y_arg,
            camera_z_arg,
            LogInfo(msg="=== Phase 1 求生本能：单目深度 → 3D 点云 → 地面过滤 ==="),
            LogInfo(msg="depth_to_cloud_node  → /camera/depth_points (camera_link)"),
            LogInfo(msg="ground_filter_node    → /depth_anything/points_filtered (camera_link)"),
            LogInfo(msg="static TF: base_link → camera_link (补全 TF 树)"),
            depth_to_cloud_node,
            ground_filter_node,
        ]
    )
