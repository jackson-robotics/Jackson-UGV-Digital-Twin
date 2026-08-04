from setuptools import find_packages, setup


package_name = "jackson_hardware"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
    ],

    install_requires=[
        "setuptools",
        "pyserial",
    ],

    zip_safe=True,

    maintainer="carlos",
    maintainer_email="carlos@todo.todo",

    description=(
        "Hardware interface for the physical Jackson robot."
    ),

    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            "esp32_odom_imu = "
            "jackson_hardware.esp32_odom_imu:main",
            "motor_driver = "
            "jackson_hardware.motor_driver:main",
        ],
    },
)
