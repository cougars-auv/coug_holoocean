import os
from glob import glob

from setuptools import find_packages, setup

package_name = "coug_holoocean"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pymap3d"],
    zip_safe=True,
    maintainer="snelsondurrant",
    maintainer_email="snelsond@byu.edu",
    description="HoloOcean simulator bridge and message converters.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
            "pytest-cov",
        ],
    },
    entry_points={
        "console_scripts": [
            "imu_converter = coug_holoocean.imu_converter_node:main",
            "gps_converter = coug_holoocean.gps_converter_node:main",
            "depth_converter = coug_holoocean.depth_converter_node:main",
            "pressure_converter = coug_holoocean.pressure_converter_node:main",
            "mag_converter = coug_holoocean.mag_converter_node:main",
            "dvl_converter = coug_holoocean.dvl_converter_node:main",
            "dvl_odom_converter = coug_holoocean.dvl_odom_converter_node:main",
            "wrench_converter = coug_holoocean.wrench_converter_node:main",
            "stereo_converter = coug_holoocean.stereo_converter_node:main",
            "truth_converter = coug_holoocean.truth_converter_node:main",
            "fin_state_publisher = coug_holoocean.fin_state_publisher_node:main",
            "cmd_vel_converter = coug_holoocean.cmd_vel_converter_node:main",
            "hsd_converter = coug_holoocean.hsd_converter_node:main",
            "modem_status_converter = coug_holoocean.modem_status_converter_node:main",
            "modem_converter = coug_holoocean.modem_converter_node:main",
        ],
    },
)
