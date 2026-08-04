from setuptools import find_packages, setup

package_name = 'jackson_waypoint_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlos',
    maintainer_email='carlos@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'waypoint_follower = jackson_waypoint_control.waypoint_follower:main',
        'waypoint_controller = jackson_waypoint_control.waypoint_controller:main',
        'physical_l_controller = jackson_waypoint_control.physical_l_controller:main',
        'physical_square_controller = jackson_waypoint_control.physical_square_controller:main',
        'physical_waypoint_controller = jackson_waypoint_control.physical_waypoint_controller:main',
        'physical_square_simple = jackson_waypoint_control.physical_square_simple:main',
    ],
},
)
