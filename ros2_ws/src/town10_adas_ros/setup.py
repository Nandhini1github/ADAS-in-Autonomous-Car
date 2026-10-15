from glob import glob
from setuptools import find_packages, setup

package_name = "town10_adas_ros"
setup(
    name=package_name, version="0.2.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="ADAS project maintainer", maintainer_email="noreply@example.com",
    description="ROS 2 ADAS replay and shadow nodes", license="Proprietary",
    entry_points={"console_scripts": [
        "csv_replay = town10_adas_ros.csv_replay:main",
        "aeb_shadow = town10_adas_ros.aeb_shadow:main",
        "metrics_recorder = town10_adas_ros.metrics_recorder:main",
    ]},
)
