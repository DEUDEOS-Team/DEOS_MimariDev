from setuptools import find_packages, setup

package_name = "deos_algorithms"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    python_requires=">=3.12",
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aaltindas",
    maintainer_email="aaltindas.work@gmail.com",
    description="Robotaksi pure-Python algorithm library",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)

