import os
from glob import glob

from setuptools import setup


package_name = "mycobot_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "urdf", "mecharm_270_m5"),
            glob("urdf/mecharm_270_m5/*"),
        ),
        (
            os.path.join("share", package_name, "urdf", "adaptive_gripper"),
            glob("urdf/adaptive_gripper/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotics lab",
    maintainer_email="student@example.com",
    description="Minimal mechArm 270 meshes for the fixed-pick submission.",
    license="BSD",
)
