"""Launch the default DA3 PointCloud2 local-planning pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("sanitation_bringup"), "config", "da3_video_params.yaml"
    )
    local_config = os.path.join(
        get_package_share_directory("sanitation_bringup"), "config", "local_avoidance.yaml"
    )
    video_path = LaunchConfiguration("video_path")
    focal_length_px = LaunchConfiguration("focal_length_px")
    model_path = LaunchConfiguration("model_path")
    use_gpu_ground_filter = LaunchConfiguration("use_gpu_ground_filter")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    use_sim_time = LaunchConfiguration("use_sim_time")

    camera = Node(
        package="sanitation_perception",
        executable="mock_camera_node",
        name="mock_camera_node",
        output="screen",
        parameters=[config, {
            "video_path": video_path,
            "focal_length_px": focal_length_px,
            "use_sim_time": use_sim_time,
        }],
    )
    da3 = Node(
        package="depth_anything_v3",
        executable="depth_anything_v3_main",
        name="depth_anything_v3",
        output="screen",
        remappings=[
            ("~/input/image", "/camera/image_raw"),
            ("~/input/camera_info", "/camera/camera_info"),
            ("~/output/depth_image", "/depth_anything_v3/output/depth_image"),
            ("~/output/point_cloud", "/depth_anything_v3/output/point_cloud"),
            ("~/output/filtered_point_cloud", "/depth_anything/points_filtered"),
            ("~/output/depth_image_debug", "/depth_anything_v3/output/depth_image_debug"),
        ],
        parameters=[config, {
            "onnx_path": model_path,
            "enable_gpu_ground_filter": ParameterValue(use_gpu_ground_filter, value_type=bool),
            "enable_point_cloud": ParameterValue(enable_point_cloud, value_type=bool),
            "use_sim_time": use_sim_time,
        }],
    )
    ground_filter = Node(
        package="sanitation_perception",
        executable="ground_filter_node",
        name="ground_filter_node",
        output="screen",
        condition=UnlessCondition(use_gpu_ground_filter),
        parameters=[config, {"use_sim_time": use_sim_time}],
    )
    costmap = Node(
        package="sanitation_navigation",
        executable="local_costmap_builder_node",
        name="local_costmap_builder_node",
        output="screen",
        parameters=[local_config, {"use_sim_time": use_sim_time}],
    )
    planner = Node(
        package="sanitation_navigation",
        executable="local_astar_planner_node",
        name="local_astar_planner_node",
        output="screen",
        parameters=[local_config, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "video_path",
            default_value="/home/ubuntu/bl/workspace/robot-dog/data/videos/test_video.mp4",
        ),
        DeclareLaunchArgument("focal_length_px", default_value="960.0"),
        DeclareLaunchArgument("model_path", default_value="models/DA3METRIC-LARGE.fp16-batch1.engine"),
        DeclareLaunchArgument("use_gpu_ground_filter", default_value="true"),
        DeclareLaunchArgument("enable_point_cloud", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        LogInfo(msg="Default pipeline: camera -> DA3 GPU filtered cloud -> costmap -> A*."),
        camera,
        TimerAction(period=1.0, actions=[da3, ground_filter, costmap, planner]),
    ])
