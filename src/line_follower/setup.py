from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ankith',
    maintainer_email='ankith@todo.todo',
    description='Line following with camera and obstacle avoidance using TurtleBot3',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'supervisor = line_follower.supervisor:main',
            'line_detector = line_follower.line_detector:main',
            'line_controller = line_follower.controller:main',
        ],
    },
)
