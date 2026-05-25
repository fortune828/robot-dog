from setuptools import find_packages, setup

package_name = "sanitation_perception"

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
    description="Visual perception nodes for sanitation robot",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'mock_camera_node = sanitation_perception.mock_camera_node:main',
            'detection_node = sanitation_perception.detection_node:main',
            'depth_to_scan_node = sanitation_perception.depth_to_scan_node:main',
            'depth_to_cloud_node = sanitation_perception.depth_to_cloud_node:main',
            'ground_filter_node = sanitation_perception.ground_filter_node:main',
        ],
    },
)
