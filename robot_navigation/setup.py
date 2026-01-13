from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share",package_name,"launch"),glob("launch/*")),
        (os.path.join("share",package_name,"config"),glob("config/*")),
        (os.path.join("share",package_name,"rviz"),glob("rviz/*")),
        (os.path.join("share",package_name,"map"),glob("map/*")),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yash',
    maintainer_email='yash@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'way = robot_navigation.waypoint:main'
        ],
    },
)
