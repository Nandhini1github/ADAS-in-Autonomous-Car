from setuptools import find_packages, setup

package_name = "town10_adas_monitor"

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
    maintainer="ADAS project maintainer",
    maintainer_email="noreply@example.com",
    description="ROS 2 status monitor for the Town10 ADAS scenario",
    license="Proprietary",
    entry_points={"console_scripts": ["vehicle_monitor = town10_adas_monitor.vehicle_monitor:main"]},
)
