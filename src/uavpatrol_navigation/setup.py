from setuptools import find_packages, setup

package_name = "uavpatrol_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "shapely", "networkx", "osmnx"],
    zip_safe=True,
    maintainer="uavpatrol",
    maintainer_email="uavpatrol@example.com",
    description="UAV patrol navigation, waypoint export, and DJI mission conversion",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mock_gps_node = uavpatrol_navigation.mock_gps_node:main",
            "osm_map_manager = uavpatrol_navigation.osm_map_manager:main",
            "polygon_coverage_planner = uavpatrol_navigation.polygon_coverage_planner:main",
            "uav_waypoint_exporter_node = uavpatrol_navigation.uav_waypoint_exporter_node:main",
            "uav_waypoints_to_dji_mission_converter = uavpatrol_navigation.uav_waypoints_to_dji_mission_converter:main",
            "demo_polygon_node = uavpatrol_navigation.demo_polygon_node:main",
        ],
    },
)
