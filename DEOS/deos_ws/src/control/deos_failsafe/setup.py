from setuptools import find_packages, setup

package_name = "deos_failsafe"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaltindas",
    maintainer_email="aaltindas.work@gmail.com",
    description="DEOS fail-safe supervisor (FSM, health, ROS integration).",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "failsafe_supervisor_node = deos_failsafe.failsafe_supervisor_node:main",
        ],
    },
)
