from setuptools import find_packages, setup

import os
from glob import glob

package_name = "dip_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/lqr_gain.csv"]),
        # Install launch files
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # Install config files
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="samuel",
    maintainer_email="samuelbruin0618@g.ucla.edu",
    description="TODO: Package description",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lqr_node = dip_control.lqr_node:main",
            "lqr_opt = dip_control.lqr_opt:main",
            "supervisor_node = dip_control.supervisor_node:main",
        ],
    },
)
