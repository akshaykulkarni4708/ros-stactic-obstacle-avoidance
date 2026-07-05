from setuptools import setup, find_packages

package_name = 'tb3_safety'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/obstacle_avoid.launch.py',
        ]),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ankith',
    maintainer_email='ankithreddy580@gmail.com',
    description='Safety node for TurtleBot3 (obstacle avoid using LaserScan)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoid = tb3_safety.obstacle_avoid:main',
        ],
    },
)
