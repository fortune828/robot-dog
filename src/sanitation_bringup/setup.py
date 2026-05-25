import os
from glob import glob
from setuptools import find_packages, setup

package_name = "sanitation_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # ====== 加上下面这两行，让编译器把 launch 和 config 复制过去 ======
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # ====================================================================
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotdog",
    maintainer_email="robotdog@example.com",
    description="Launch files and configuration for sanitation robot system",
    license="Apache-2.0",
)
