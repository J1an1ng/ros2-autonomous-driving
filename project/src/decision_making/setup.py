from glob import glob
from setuptools import find_packages, setup


package_name = "decision_making"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Yida",
    maintainer_email="yida@todo.todo",
    description="Traffic-light decision state machine.",
    license="TODO",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "traffic_light_state_machine_node = "
            "decision_making.traffic_light_state_machine_node:main",
        ],
    },
)
