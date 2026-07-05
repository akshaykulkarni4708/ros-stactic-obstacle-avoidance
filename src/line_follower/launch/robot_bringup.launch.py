#!/usr/bin/env python3
"""
Bring up the line-following + obstacle-avoidance stack on a physical TurtleBot3.

Runs ON the robot's onboard computer (e.g. Raspberry Pi). Includes the
official turtlebot3_bringup base/IMU/LiDAR driver and Pi camera driver,
then starts this project's detector/controller/obstacle-avoid/supervisor
nodes with use_sim_time:=false.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_turtlebot3_bringup = get_package_share_directory('turtlebot3_bringup')
    pkg_line_follower = get_package_share_directory('line_follower')

    turtlebot3_model = LaunchConfiguration('turtlebot3_model')
    lds_model = LaunchConfiguration('lds_model')
    usb_port = LaunchConfiguration('usb_port')

    # turtlebot3_bringup/launch/robot.launch.py reads TURTLEBOT3_MODEL and
    # LDS_MODEL straight from the process environment (same constraint as
    # spawn_turtlebot3.launch.py in simulation), so set them before it loads.
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model)
    set_lds_model = SetEnvironmentVariable('LDS_MODEL', lds_model)

    robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_bringup, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'usb_port': usb_port,
        }.items(),
    )

    camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_bringup, 'launch', 'camera.launch.py')
        ),
    )

    line_follow_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_line_follower, 'launch', 'line_follow.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'turtlebot3_model', default_value='waffle_pi',
            description='TurtleBot3 model (burger, waffle, waffle_pi) -- '
                        'must match the physical robot'),
        DeclareLaunchArgument(
            'lds_model', default_value='LDS-01',
            description='LiDAR model fitted to the robot (LDS-01, LDS-02, LDS-03)'),
        DeclareLaunchArgument(
            'usb_port', default_value='/dev/ttyACM0',
            description='USB serial port for the OpenCR board'),
        set_tb3_model,
        set_lds_model,
        robot_cmd,
        camera_cmd,
        line_follow_cmd,
    ])
