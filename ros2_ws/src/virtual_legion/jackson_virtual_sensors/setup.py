from setuptools import find_packages, setup


package_name = "jackson_virtual_sensors"


setup(
    name=package_name,
    version="0.0.0",

    packages=find_packages(
        exclude=["test"]
    ),

    data_files=[
        (
            "share/ament_index/"
            "resource_index/packages",
            [
                "resource/"
                + package_name
            ],
        ),
        (
            "share/" + package_name,
            [
                "package.xml",
            ],
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="carlos",
    maintainer_email=(
        "carlos@todo.todo"
    ),

    description=(
        "Virtual wheel odometry "
        "for the Jackson Isaac Sim model."
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "joint_state_odometry = "
                "jackson_virtual_sensors."
                "joint_state_odometry:main"
            ),
            (
                "imu_preprocessor = "
                "jackson_virtual_sensors."
                "imu_preprocessor:main"
            ),
        ],
    },
)
