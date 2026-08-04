from setuptools import find_packages, setup


package_name = "jackson_experiments"


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
    ],

    zip_safe=True,

    maintainer="carlos",
    maintainer_email="carlos@todo.todo",

    description=(
        "Reproducible experiments for the "
        "Jackson digital twin validation."
    ),

    license="Apache-2.0",

    entry_points={
        "console_scripts": [
            (
                "straight_distance = "
                "jackson_experiments.straight_distance:main"
            ),
            (
                "square_1m = "
                "jackson_experiments.square_1m:main"
            ),
            (
                "analyze_square_bag = "
                "jackson_experiments.analyze_square_bag:main"
            ),
        ],
    },
)
