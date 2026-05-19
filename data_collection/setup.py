from setuptools import find_packages, setup

package_name = "data_collection"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/arm_teleop.launch.py",
                "launch/data_collector.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Frank Zillmann",
    maintainer_email="frank.zillmann@tum.de",
    description="Simple data collection and arm teleop tools for Mirte Master",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_teleop = data_collection.arm_teleop:main",
            "data_collector = data_collection.data_collector:main",
        ],
    },
)
