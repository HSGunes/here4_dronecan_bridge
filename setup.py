from setuptools import find_packages, setup
import os
from glob import glob

package_name = "here4_dronecan_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "scripts"),
            [f for f in glob("scripts/*") if os.path.isfile(f)],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="gunes",
    maintainer_email="hikmetselcukgunes@gmail.com",
    description="DroneCAN bridge for Here 4 GPS/IMU/Magnetometer sensor.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "here4_bridge_node = here4_dronecan_bridge.here4_bridge_node:main",
            "ntrip_test = here4_dronecan_bridge.ntrip_test_cli:main",
            "set_gps_l5 = here4_dronecan_bridge.set_gps_l5:main",
        ],
    },
)
