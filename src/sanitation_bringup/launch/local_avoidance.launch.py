"""local_avoidance.launch.py — 局部避障验证：深度点云 → 地面过滤 → Nav2 局部代价地图

Launches:
  1. mock_gps_node           — world → odom TF (identity, idle)
  2. mock_chassis_node       — odom → base_link TF (+ /odom)
  3. depth_to_cloud_node     — Depth Anything V2 → /camera/depth_points
  4. ground_filter_node      — Z 轴过滤 → /depth_anything/points_filtered
                              + static TF base_link → camera_link

然后手动启动 Nav2 (见下方注释) 或使用 nav2_bringup:
  ros2 launch nav2_bringup bringup_launch.py \
    params_file:=src/sanitation_bringup/config/nav2_local_params.yaml \
    use_sim_time:=false

RViz2 中关键观察项:
  - /local_costmap/costmap       — 障碍物红/青色膨胀区
  - /depth_anything/points_filtered — 悬空障碍物点云 (Z 0.15~0.80m)
  - /local_plan                   — 局部绕障路径
  - TF 树: world → odom → base_link → camera_link 完整无断链
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_path_arg = DeclareLaunchArgument(
        "video_path", default_value="",
        description="MP4 video file",
    )
    stride_arg = DeclareLaunchArgument(
        "stride", default_value="4",
        description="Point cloud downsampling",
    )
    min_z_arg = DeclareLaunchArgument(
        "min_z", default_value="0.15",
        description="Z floor threshold (m)",
    )
    max_z_arg = DeclareLaunchArgument(
        "max_z", default_value="0.80",
        description="Z ceiling threshold (m)",
    )
    device_arg = DeclareLaunchArgument(
        "device", default_value="auto",
        description="'cuda', 'cpu', or 'auto'",
    )

    video_path = LaunchConfiguration("video_path")
    stride = LaunchConfiguration("stride")
    min_z = LaunchConfiguration("min_z")
    max_z = LaunchConfiguration("max_z")
    device = LaunchConfiguration("device")

    config_dir = os.path.join(
        get_package_share_directory("sanitation_bringup"), "config"
    )
    default_config = os.path.join(config_dir, "default_params.yaml")

    # ---- TF 基础设施 ----
    mock_gps_node = Node(
        package="sanitation_navigation",
        executable="mock_gps_node",
        name="mock_gps_node",
        output="screen",
        parameters=[default_config],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    mock_chassis_node = Node(
        package="sanitation_navigation",
        executable="mock_chassis_node",
        name="mock_chassis_node",
        output="screen",
        parameters=[default_config],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # ---- 感知管线 ----
    depth_to_cloud_node = Node(
        package="sanitation_perception",
        executable="depth_to_cloud_node",
        name="depth_to_cloud_node",
        output="screen",
        parameters=[default_config, {
            "stride": stride,
            "device": device,
            "video_path": video_path,
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    ground_filter_node = Node(
        package="sanitation_perception",
        executable="ground_filter_node",
        name="ground_filter_node",
        output="screen",
        parameters=[default_config, {
            "min_z": min_z,
            "max_z": max_z,
        }],
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription([
        video_path_arg, stride_arg, min_z_arg, max_z_arg, device_arg,
        LogInfo(msg="=== 局部避障验证：TF 基础设施 ==="),
        LogInfo(msg="mock_gps_node     → world → odom TF (idle)"),
        LogInfo(msg="mock_chassis_node → odom → base_link TF"),
        LogInfo(msg=""),
        LogInfo(msg="=== 局部避障验证：感知管线 ==="),
        LogInfo(msg="depth_to_cloud  → /camera/depth_points (raw)"),
        LogInfo(msg="ground_filter   → /depth_anything/points_filtered (Z %.2f~%.2f)" % (0.15, 0.80)),
        LogInfo(msg=""),
        LogInfo(msg="=== 手动启动 Nav2 (新终端): ==="),
        LogInfo(msg="ros2 launch nav2_bringup bringup_launch.py "
                "params_file:=src/sanitation_bringup/config/nav2_local_params.yaml "
                "use_sim_time:=false"),
        mock_gps_node,
        mock_chassis_node,
        depth_to_cloud_node,
        ground_filter_node,
    ])
