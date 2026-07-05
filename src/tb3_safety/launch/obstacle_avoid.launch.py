from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_obstacle'),
        DeclareLaunchArgument('front_half_angle_deg', default_value='25.0'),
        DeclareLaunchArgument('side_sector_max_deg', default_value='170.0'),
        DeclareLaunchArgument('lidar_to_hull_margin', default_value='0.20'),
        DeclareLaunchArgument('avoid_distance', default_value='0.80'),
        DeclareLaunchArgument('emergency_distance', default_value='0.25'),
        DeclareLaunchArgument('stop_time_sec', default_value='0.60'),
        DeclareLaunchArgument('back_off_speed', default_value='0.12'),
        DeclareLaunchArgument('back_off_growth_sec', default_value='0.40'),
        DeclareLaunchArgument('back_off_max_time_sec', default_value='5.00'),
        DeclareLaunchArgument('turn_direction_hysteresis_m', default_value='0.10'),
        DeclareLaunchArgument('turn_time_sec', default_value='2.30'),
        DeclareLaunchArgument('turn_speed', default_value='0.30'),
        DeclareLaunchArgument('forward_speed', default_value='0.12'),
        DeclareLaunchArgument('forward_distance_m', default_value='2.20'),
        DeclareLaunchArgument('forward_time_sec', default_value='15.00'),
        DeclareLaunchArgument('yaw_tolerance_deg', default_value='5.0'),
        DeclareLaunchArgument('search_turn_speed', default_value='0.12'),
        DeclareLaunchArgument('line_search_error_threshold', default_value='20.0'),
        DeclareLaunchArgument('line_search_confirm_count', default_value='8'),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
    ]

    avoid_node = Node(
        package='tb3_safety',
        executable='obstacle_avoid',
        name='obstacle_avoid',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'front_half_angle_deg': LaunchConfiguration('front_half_angle_deg'),
            'side_sector_max_deg': LaunchConfiguration('side_sector_max_deg'),
            'lidar_to_hull_margin': LaunchConfiguration('lidar_to_hull_margin'),
            'avoid_distance': LaunchConfiguration('avoid_distance'),
            'emergency_distance': LaunchConfiguration('emergency_distance'),
            'stop_time_sec': LaunchConfiguration('stop_time_sec'),
            'back_off_speed': LaunchConfiguration('back_off_speed'),
            'back_off_growth_sec': LaunchConfiguration('back_off_growth_sec'),
            'back_off_max_time_sec': LaunchConfiguration('back_off_max_time_sec'),
            'turn_direction_hysteresis_m': LaunchConfiguration('turn_direction_hysteresis_m'),
            'turn_time_sec': LaunchConfiguration('turn_time_sec'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'forward_speed': LaunchConfiguration('forward_speed'),
            'forward_distance_m': LaunchConfiguration('forward_distance_m'),
            'forward_time_sec': LaunchConfiguration('forward_time_sec'),
            'yaw_tolerance_deg': LaunchConfiguration('yaw_tolerance_deg'),
            'search_turn_speed': LaunchConfiguration('search_turn_speed'),
            'line_search_error_threshold': LaunchConfiguration('line_search_error_threshold'),
            'line_search_confirm_count': LaunchConfiguration('line_search_confirm_count'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
        }],
    )

    return LaunchDescription(arguments + [avoid_node])
