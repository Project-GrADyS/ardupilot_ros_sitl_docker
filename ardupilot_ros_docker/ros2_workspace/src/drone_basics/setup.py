from setuptools import find_packages, setup

package_name = 'drone_basics'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_client = drone_basics.ArmTest:main',
            'takeoff_land_node = drone_basics.TakeoffLand:main',
            'line_node = drone_basics.Line:main',
            'random_waypoint_node = drone_basics.RandomWaypoint:main',
        ],
    },
)
