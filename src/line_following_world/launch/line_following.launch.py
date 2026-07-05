#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3 = get_package_share_directory('turtlebot3_gazebo')
    pkg_line_follower = get_package_share_directory('line_follower')
    pkg_line_following_world = get_package_share_directory('line_following_world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model')
    world_file = LaunchConfiguration('world_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # spawn_turtlebot3.launch.py (turtlebot3_gazebo) reads TURTLEBOT3_MODEL
    # straight out of the process environment, so it must be set before that
    # launch file is included, not just declared as a ROS parameter.
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model)

    world = PathJoinSubstitution([pkg_line_following_world, 'worlds', world_file])
    rviz_config = os.path.join(pkg_line_following_world, 'rviz', 'line_follower.rviz')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    line_follower_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_line_follower, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument(
            'turtlebot3_model', default_value='waffle_pi',
            description='TurtleBot3 model to spawn (burger, waffle, waffle_pi)'),
        DeclareLaunchArgument(
            'world_file', default_value='line_following.world',
            description=(
                'World file (in line_following_world/worlds) to load: '
                'line_following.world (straight line) or '
                'line_world.world (curved line + second obstacle layout)'
            )),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Launch RViz2 alongside the Gazebo GUI'),
        set_tb3_model,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        line_follower_cmd,
        rviz_cmd,
    ])

    return ld
