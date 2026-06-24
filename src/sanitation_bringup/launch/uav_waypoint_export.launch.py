"""Launch the UAV GPS waypoint planning/export pipeline.

This launch keeps the existing OSM and coverage planners, but replaces the
ground robot control layer with a plain GPS waypoint exporter for DJI UAV tests.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="false")
    use_mock_gps_arg = DeclareLaunchArgument(
        "use_mock_gps",
        default_value="false",
        description="Publish simulated /fix for desktop tests.",
    )
    gps_topic_arg = DeclareLaunchArgument(
        "gps_topic",
        default_value="/fix",
        description="RTK GPS topic. For DJI tests, remap/provide this topic.",
    )
    altitude_arg = DeclareLaunchArgument(
        "altitude_m",
        default_value="30.0",
        description="Default UAV waypoint altitude in meters.",
    )
    altitude_mode_arg = DeclareLaunchArgument(
        "altitude_mode",
        default_value="relative_to_takeoff",
        description="relative_to_takeoff or absolute_amsl.",
    )
    speed_arg = DeclareLaunchArgument(
        "speed_mps",
        default_value="5.0",
        description="Default UAV waypoint speed in m/s.",
    )
    coordinate_frame_arg = DeclareLaunchArgument(
        "coordinate_frame",
        default_value="WGS84",
        description="WGS84 by default; CGCS2000 can be selected if the UAV app expects it.",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="/home/ubuntu/bl/workspace/robot-dog/data/output",
        description="Directory for exported UAV waypoint CSV/JSON files.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_gps = LaunchConfiguration("use_mock_gps")
    gps_topic = LaunchConfiguration("gps_topic")
    altitude_m = LaunchConfiguration("altitude_m")
    altitude_mode = LaunchConfiguration("altitude_mode")
    speed_mps = LaunchConfiguration("speed_mps")
    coordinate_frame = LaunchConfiguration("coordinate_frame")
    output_dir = LaunchConfiguration("output_dir")

    config_dir = os.path.join(
        get_package_share_directory("sanitation_bringup"), "config"
    )
    default_config = os.path.join(config_dir, "default_params.yaml")

    mock_gps_node = Node(
        package="sanitation_navigation",
        executable="mock_gps_node",
        name="mock_gps_node",
        output="screen",
        condition=IfCondition(use_mock_gps),
        parameters=[
            default_config,
            {
                "topic": gps_topic,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    osm_map_manager = Node(
        package="sanitation_navigation",
        executable="osm_map_manager",
        name="osm_map_manager",
        output="screen",
        parameters=[
            default_config,
            {
                "gps_topic": gps_topic,
                "origin_from_first_fix": True,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    polygon_coverage_planner = Node(
        package="sanitation_navigation",
        executable="polygon_coverage_planner",
        name="polygon_coverage_planner",
        output="screen",
        parameters=[
            default_config,
            {
                "gps_topic": gps_topic,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    uav_waypoint_exporter = Node(
        package="sanitation_navigation",
        executable="uav_waypoint_exporter_node",
        name="uav_waypoint_exporter_node",
        output="screen",
        parameters=[
            default_config,
            {
                "gps_topic": gps_topic,
                "origin_mode": "first_fix",
                "altitude_m": altitude_m,
                "altitude_mode": altitude_mode,
                "speed_mps": speed_mps,
                "coordinate_frame": coordinate_frame,
                "output_dir": output_dir,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_mock_gps_arg,
            gps_topic_arg,
            altitude_arg,
            altitude_mode_arg,
            speed_arg,
            coordinate_frame_arg,
            output_dir_arg,
            LogInfo(msg="=== UAV waypoint export mode ==="),
            LogInfo(msg="Input: /fix + /goal_pose or /clicked_point"),
            LogInfo(msg="Output: /global_plan -> data/output/uav_waypoints.csv/json"),
            mock_gps_node,
            osm_map_manager,
            polygon_coverage_planner,
            uav_waypoint_exporter,
        ]
    )
