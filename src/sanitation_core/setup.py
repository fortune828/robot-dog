from setuptools import find_packages, setup

package_name = "sanitation_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="robotdog",
    maintainer_email="robotdog@example.com",
    description="Pure Python core algorithms for sanitation robot",
    license="Apache-2.0",
)
