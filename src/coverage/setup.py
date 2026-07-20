import os
from glob import glob

from setuptools import find_packages, setup

package_name = "coverage"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jtao",
    maintainer_email="tj19970215@gmail.com",
    description="Camera-view coverage: nearest-unseen goal selection for automatic semantic mapping.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "coverage_node = coverage.coverage_node:main",
        ],
    },
)
