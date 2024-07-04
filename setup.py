from setuptools import find_packages, setup

package_name = "rc_automobile_controller"

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
    maintainer="george",
    maintainer_email="ngigegeorge023@gmail.com",
    description="ROS controller for the differential rover. Refer the README file.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "test_node = rc_automobile_controller.test_node:main",
            "rc_automobile_controller = rc_automobile_controller.main:main",
        ],
    },
)
