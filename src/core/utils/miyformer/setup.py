from setuptools import find_packages, setup
import os
from glob import glob

package_name = "miyformer"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="A simple ROS2 Python package with PyTorch integration",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "miyformer_node = miyformer.miyformer_node:main",
        ],
    },
)
