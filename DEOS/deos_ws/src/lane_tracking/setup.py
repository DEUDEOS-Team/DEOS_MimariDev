from setuptools import find_packages, setup

package_name = "lane_tracking"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@todo.local",
    description="Lane detection + lane control nodes (Hailo/CPU) for Raspberry.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lane_detection_node = lane_tracking.lane_detection_node:main",
            "lane_control_node = lane_tracking.lane_control_node:main",
        ],
    },
)

