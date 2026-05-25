from setuptools import find_packages, setup

package_name = "sanitation_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotdog",
    maintainer_email="robotdog@example.com",
    description="Navigation and chassis control nodes for sanitation robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'mock_chassis_node = sanitation_navigation.mock_chassis_node:main',
            'waypoint_patrol_node = sanitation_navigation.waypoint_patrol_node:main',
            'mock_gps_node = sanitation_navigation.mock_gps_node:main',
            'gaode_path_proxy = sanitation_navigation.gaode_path_proxy:main',
            'polygon_coverage_planner = sanitation_navigation.polygon_coverage_planner:main',
            'osm_map_manager = sanitation_navigation.osm_map_manager:main',
        ],
    },
)
