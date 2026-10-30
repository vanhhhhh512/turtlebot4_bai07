from setuptools import setup, find_packages
from glob import glob
import os

package_name = "bai07_turtlebot4"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name,
            ["package.xml", "README.md", "LICENSE", "NOTICE", "requirements-asr.txt"],
        ),
    ]
    + [
        (
            os.path.join("share", package_name, d),
            [f for f in glob(d + "/*") if os.path.isfile(f)],
        )
        for d in ["maps", "config", "launch", "rviz", "scripts"]
    ],
    install_requires=["setuptools", "PyYAML", "numpy"],
    zip_safe=True,
    maintainer="Đỗ Việt Anh",
    maintainer_email="23020719@vnu.edu.vn",
    description="Vietnamese speech to semantic maze goals and Nav2 on TurtleBot 4",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            n + " = bai07_turtlebot4." + n + ":main"
            for n in [
                "map_segmentation",
                "semantic_map_node",
                "voice_navigation_node",
                "speech_recognition_node",
                "demo_panel",
                "prepare_robot",
                "evidence_recorder",
            ]
        ]
    },
)
