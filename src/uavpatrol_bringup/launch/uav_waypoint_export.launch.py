"""Launch the UAV GPS waypoint planning/export pipeline.

This launch keeps the OSM and coverage planners, then exports GPS waypoints
for DJI UAV patrol mission generation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from uavpatrol_navigation.planning_defaults import DEFAULT_PLANNING


def _default(value):
    return str(value)


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
        default_value=_default(DEFAULT_PLANNING.altitude_m),
        description="Default UAV waypoint altitude in meters.",
    )
    altitude_mode_arg = DeclareLaunchArgument(
        "altitude_mode",
        default_value=DEFAULT_PLANNING.altitude_mode,
        description="relative_to_takeoff or absolute_amsl.",
    )
    speed_arg = DeclareLaunchArgument(
        "speed_mps",
        default_value=_default(DEFAULT_PLANNING.speed_mps),
        description="Default UAV waypoint speed in m/s.",
    )
    sweep_spacing_arg = DeclareLaunchArgument(
        "sweep_spacing",
        default_value=_default(DEFAULT_PLANNING.coverage_spacing_m),
        description="Coverage sweep spacing in meters. Use smaller values for small test areas.",
    )
    route_edge_margin_arg = DeclareLaunchArgument(
        "route_edge_margin",
        default_value=_default(DEFAULT_PLANNING.route_edge_margin_m),
        description="Prefer route points this many meters inside flyable area when possible.",
    )
    max_off_area_distance_arg = DeclareLaunchArgument(
        "max_off_area_distance",
        default_value=_default(DEFAULT_PLANNING.max_off_area_distance_m),
        description="Allow transit this many meters outside the patrol area boundary.",
    )
    enable_semantic_constraints_arg = DeclareLaunchArgument(
        "enable_osm_semantic_constraints",
        default_value="true",
        description="Use OSM building buffers as static no-fly constraints.",
    )
    coordinate_frame_arg = DeclareLaunchArgument(
        "coordinate_frame",
        default_value="WGS84",
        description="WGS84 by default; CGCS2000 can be selected if the UAV app expects it.",
    )
    output_dir_arg = DeclareLaunchArgument(
        "output_dir",
        default_value="",
        description="Directory for exported UAV waypoint CSV/JSON files.",
    )
    area_source_arg = DeclareLaunchArgument(
        "area_source",
        default_value="file",
        description="Patrol area source: file, clicked_point, or demo.",
    )
    patrol_area_file_arg = DeclareLaunchArgument(
        "patrol_area_file",
        default_value=DEFAULT_PLANNING.patrol_area_file,
        description="Patrol area JSON file, relative to the project root unless absolute.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock_gps = LaunchConfiguration("use_mock_gps")
    gps_topic = LaunchConfiguration("gps_topic")
    altitude_m = LaunchConfiguration("altitude_m")
    altitude_mode = LaunchConfiguration("altitude_mode")
    speed_mps = LaunchConfiguration("speed_mps")
    sweep_spacing = LaunchConfiguration("sweep_spacing")
    route_edge_margin = LaunchConfiguration("route_edge_margin")
    max_off_area_distance = LaunchConfiguration("max_off_area_distance")
    enable_semantic_constraints = LaunchConfiguration("enable_osm_semantic_constraints")
    coordinate_frame = LaunchConfiguration("coordinate_frame")
    output_dir = LaunchConfiguration("output_dir")
    area_source = LaunchConfiguration("area_source")
    patrol_area_file = LaunchConfiguration("patrol_area_file")

    config_dir = os.path.join(
        get_package_share_directory("uavpatrol_bringup"), "config"
    )
    default_config = os.path.join(config_dir, "default_params.yaml")

    mock_gps_node = Node(
        package="uavpatrol_navigation",
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
        package="uavpatrol_navigation",
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
        package="uavpatrol_navigation",
        executable="polygon_coverage_planner",
        name="polygon_coverage_planner",
        output="screen",
        parameters=[
            default_config,
            {
                "gps_topic": gps_topic,
                "area_source": area_source,
                "patrol_area_file": patrol_area_file,
                "sweep_spacing": sweep_spacing,
                "coverage_spacing_m": sweep_spacing,
                "route_edge_margin_m": route_edge_margin,
                "max_off_area_distance_m": max_off_area_distance,
                "enable_osm_semantic_constraints": enable_semantic_constraints,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    uav_waypoint_exporter = Node(
        package="uavpatrol_navigation",
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
                "patrol_area_file": patrol_area_file,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    demo_polygon_node = Node(
        package="uavpatrol_navigation",
        executable="demo_polygon_node",
        name="demo_polygon_node",
        output="screen",
        condition=IfCondition(PythonExpression(["'", area_source, "' == 'demo'"])),
        parameters=[default_config, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            use_mock_gps_arg,
            gps_topic_arg,
            altitude_arg,
            altitude_mode_arg,
            speed_arg,
            sweep_spacing_arg,
            route_edge_margin_arg,
            max_off_area_distance_arg,
            enable_semantic_constraints_arg,
            coordinate_frame_arg,
            output_dir_arg,
            area_source_arg,
            patrol_area_file_arg,
            LogInfo(msg="=== UAV waypoint export mode ==="),
            LogInfo(msg="Input: patrol_area.json or /clicked_point + /fix"),
            LogInfo(msg="Planner: polygon_coverage_planner -> /global_plan"),
            LogInfo(msg="Output: data/output/uav_waypoints + DJI mission files"),
            mock_gps_node,
            osm_map_manager,
            polygon_coverage_planner,
            uav_waypoint_exporter,
            demo_polygon_node,
        ]
    )
