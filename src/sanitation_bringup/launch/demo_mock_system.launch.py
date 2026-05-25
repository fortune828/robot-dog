"""demo_mock_system.launch.py — 一键启动全 Mock 闭环系统。

启动节点:
  1. mock_camera_node          — 模拟摄像头（发布 /camera/image_raw）
  2. detection_node            — 目标检测 [WARN 模式]
  3. mock_chassis_node         — 模拟底盘 [WARN 模式]
  4. mock_gps_node             — 模拟 RTK GPS（空闲噪声 / 收到 Path 后沿路移动）
  5. osm_map_manager           — OSM 地图引擎（路网可视化 + 离线路由 /plan_osm_path）
  6. polygon_coverage_planner  — 全覆盖弓字形路径规划（/clicked_point → /global_plan）

双轨数据流:
  通勤链路: /goal_pose → osm_map_manager → /global_plan → mock_gps_node → /fix
            polygon_coverage_planner → /plan_osm_path → osm_map_manager → 通勤段拼接
  覆盖链路: /clicked_point → polygon_coverage_planner → /global_plan → mock_gps_node → /fix
  感知链路: mock_camera → detection_node (背景运行，WARN 静默)

用法:
  ros2 launch sanitation_bringup demo_mock_system.launch.py
  ros2 launch sanitation_bringup demo_mock_system.launch.py video_path:=/path/to/video.mp4
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_data_dir() -> str:
    """查找项目根目录下的 data/"""
    p = Path(__file__).resolve()
    for parent in [p] + list(p.parents):
        candidate = parent / "data"
        if candidate.is_dir():
            return str(candidate)
    return ""


def generate_launch_description():
    # ---- 参数声明 ----
    video_path_arg = DeclareLaunchArgument(
        "video_path",
        default_value="",
        description="测试视频路径，留空则自动查找 data/test_video.mp4 或生成合成图像",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="是否使用仿真时间",
    )

    video_path = LaunchConfiguration("video_path")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # ---- 公共配置 ----
    config_dir = os.path.join(
        get_package_share_directory("sanitation_bringup"), "config"
    )
    default_config = os.path.join(config_dir, "default_params.yaml")

    # ---- 节点定义 ----

    # 1. 模拟摄像头节点
    mock_camera_node = Node(
        package="sanitation_perception",
        executable="mock_camera_node",
        name="mock_camera_node",
        output="screen",
        parameters=[default_config, {"video_path": video_path}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 2. 目标检测节点（静默模式）
    detection_node = Node(
        package="sanitation_perception",
        executable="detection_node",
        name="detection_node",
        output="screen",
        parameters=[default_config],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # 3. 模拟底盘节点（静默模式）
    mock_chassis_node = Node(
        package="sanitation_navigation",
        executable="mock_chassis_node",
        name="mock_chassis_node",
        output="screen",
        parameters=[default_config],
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # 4. 模拟 RTK GPS 节点
    mock_gps_node = Node(
        package="sanitation_navigation",
        executable="mock_gps_node",
        name="mock_gps_node",
        output="screen",
        parameters=[default_config],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 5. 路点巡检节点 — 高德模式下已禁用
    #     (mock_gps_node 通过 /global_plan 直接控制位置)

    # 6. OSM 地图引擎 — 路网可视化 + 离线路由服务
    osm_map_manager = Node(
        package="sanitation_navigation",
        executable="osm_map_manager",
        name="osm_map_manager",
        output="screen",
        parameters=[default_config],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 7. 全覆盖路径规划 (CPP) — 弓字形扫掠
    polygon_coverage_planner = Node(
        package="sanitation_navigation",
        executable="polygon_coverage_planner",
        name="polygon_coverage_planner",
        output="screen",
        parameters=[default_config],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ---- 启动顺序（按依赖关系延迟启动） ----
    # perception 链: camera 先启动，稍后 detection 再订阅
    # navigation 链: chassis 先启动（提供 odom），patrol 再订阅 odom 并发布 cmd_vel
    # GPS 独立运行，无需等待其他节点

    return LaunchDescription(
        [
            video_path_arg,
            use_sim_time_arg,
            LogInfo(msg="=== 四足环卫机器人 — OSM 本地拓扑导航 + 局部覆盖 双轨架构 ==="),
            LogInfo(msg="通勤链路: /goal_pose → osm_map_manager → /global_plan → mock_gps_node → /fix"),
            LogInfo(msg="覆盖链路: /clicked_point → polygon_coverage_planner → /global_plan → mock_gps_node → /fix"),
            LogInfo(msg="感知链路: mock_camera → detection_node (WARN 静默)"),
            # 数据源: camera + chassis + gps + osm_map + cpp
            mock_camera_node,
            mock_chassis_node,
            mock_gps_node,
            osm_map_manager,
            polygon_coverage_planner,
            # 延迟 1 秒启动消费节点
            TimerAction(
                period=1.0,
                actions=[
                    detection_node,
                    LogInfo(
                        msg="=== 全部节点已启动，在 RViz 中使用 2D Goal Pose 规划路径 ==="
                    ),
                ],
            ),
        ]
    )
