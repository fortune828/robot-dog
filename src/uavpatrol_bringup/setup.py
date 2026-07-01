import os
from glob import glob
from setuptools import find_packages, setup

package_name = "uavpatrol_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="uavpatrol",
    maintainer_email="uavpatrol@example.com",
    description="Launch files and configuration for UAV patrol",
    license="Apache-2.0",
)
