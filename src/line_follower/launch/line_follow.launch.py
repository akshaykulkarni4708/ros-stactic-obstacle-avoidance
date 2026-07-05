from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    env_qt = SetEnvironmentVariable(name='QT_X11_NO_MITSHM', value='1')

    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    front_half_angle_deg = LaunchConfiguration('front_half_angle_deg')
    side_sector_max_deg = LaunchConfiguration('side_sector_max_deg')
    lidar_to_hull_margin = LaunchConfiguration('lidar_to_hull_margin')
    avoid_distance = LaunchConfiguration('avoid_distance')
    emergency_distance = LaunchConfiguration('emergency_distance')
    stop_time_sec = LaunchConfiguration('stop_time_sec')
    back_off_speed = LaunchConfiguration('back_off_speed')
    back_off_growth_sec = LaunchConfiguration('back_off_growth_sec')
    back_off_max_time_sec = LaunchConfiguration('back_off_max_time_sec')
    turn_direction_hysteresis_m = LaunchConfiguration('turn_direction_hysteresis_m')
    turn_time_sec = LaunchConfiguration('turn_time_sec')
    turn_speed = LaunchConfiguration('turn_speed')
    forward_speed = LaunchConfiguration('forward_speed')
    forward_distance_m = LaunchConfiguration('forward_distance_m')
    forward_time_sec = LaunchConfiguration('forward_time_sec')
    yaw_tolerance_deg = LaunchConfiguration('yaw_tolerance_deg')
    search_turn_speed = LaunchConfiguration('search_turn_speed')
    line_search_error_threshold = LaunchConfiguration('line_search_error_threshold')
    line_search_confirm_count = LaunchConfiguration('line_search_confirm_count')
    search_max_yaw_deviation_deg = LaunchConfiguration('search_max_yaw_deviation_deg')
    search_timeout_sec = LaunchConfiguration('search_timeout_sec')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    line_topic = LaunchConfiguration('line_topic')
    avoid_topic = LaunchConfiguration('avoid_topic')
    output_topic = LaunchConfiguration('output_topic')
    rate_hz = LaunchConfiguration('rate_hz')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    roi_start_arg = DeclareLaunchArgument('roi_start', default_value='0.48')
    line_is_dark_arg = DeclareLaunchArgument('line_is_dark', default_value='true')
    use_adaptive_arg = DeclareLaunchArgument('use_adaptive', default_value='true')
    adaptive_block_arg = DeclareLaunchArgument('adaptive_block', default_value='51')
    adaptive_c_arg = DeclareLaunchArgument('adaptive_c', default_value='2')
    kernel_size_arg = DeclareLaunchArgument('kernel_size', default_value='5')
    fixed_thresh_arg = DeclareLaunchArgument('fixed_thresh', default_value='150')
    use_hsv_arg = DeclareLaunchArgument('use_hsv', default_value='true')
    hsv_lower_h_arg = DeclareLaunchArgument('hsv_lower_h', default_value='15')
    hsv_lower_s_arg = DeclareLaunchArgument('hsv_lower_s', default_value='80')
    hsv_lower_v_arg = DeclareLaunchArgument('hsv_lower_v', default_value='80')
    hsv_upper_h_arg = DeclareLaunchArgument('hsv_upper_h', default_value='40')
    hsv_upper_s_arg = DeclareLaunchArgument('hsv_upper_s', default_value='255')
    hsv_upper_v_arg = DeclareLaunchArgument('hsv_upper_v', default_value='255')
    min_nonzero_arg = DeclareLaunchArgument('min_nonzero', default_value='50')
    max_fill_ratio_arg = DeclareLaunchArgument('max_fill_ratio', default_value='0.60')
    min_contour_area_arg = DeclareLaunchArgument('min_contour_area', default_value='250.0')
    lost_sentinel_arg = DeclareLaunchArgument('lost_sentinel', default_value='-1.0')
    ema_alpha_arg = DeclareLaunchArgument('ema_alpha', default_value='0.25')
    max_contour_jump_arg = DeclareLaunchArgument('max_contour_jump', default_value='120.0')
    contour_switch_confirm_frames_arg = DeclareLaunchArgument(
        'contour_switch_confirm_frames', default_value='3')

    linear_x_arg = DeclareLaunchArgument('linear_x', default_value='0.04')
    k_p_arg = DeclareLaunchArgument('k_p', default_value='0.004')
    max_ang_z_arg = DeclareLaunchArgument('max_ang_z', default_value='0.4')
    steer_sign_arg = DeclareLaunchArgument('steer_sign', default_value='-1.0')
    search_w_arg = DeclareLaunchArgument('search_w', default_value='0.35')
    search_linear_x_arg = DeclareLaunchArgument('search_linear_x', default_value='0.02')
    min_linear_x_arg = DeclareLaunchArgument('min_linear_x', default_value='0.02')
    slowdown_error_arg = DeclareLaunchArgument('slowdown_error', default_value='80.0')
    turn_in_place_error_arg = DeclareLaunchArgument('turn_in_place_error', default_value='240.0')
    error_deadband_arg = DeclareLaunchArgument('error_deadband', default_value='10.0')
    angular_alpha_arg = DeclareLaunchArgument('angular_alpha', default_value='0.35')
    lost_timeout_sec_arg = DeclareLaunchArgument('lost_timeout_sec', default_value='6.0')

    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value='/scan')
    cmd_vel_topic_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel_obstacle')
    front_half_angle_deg_arg = DeclareLaunchArgument('front_half_angle_deg', default_value='25.0')
    side_sector_max_deg_arg = DeclareLaunchArgument('side_sector_max_deg', default_value='170.0')
    lidar_to_hull_margin_arg = DeclareLaunchArgument('lidar_to_hull_margin', default_value='0.20')
    avoid_distance_arg = DeclareLaunchArgument('avoid_distance', default_value='0.80')
    emergency_distance_arg = DeclareLaunchArgument('emergency_distance', default_value='0.25')
    stop_time_sec_arg = DeclareLaunchArgument('stop_time_sec', default_value='0.60')
    back_off_speed_arg = DeclareLaunchArgument('back_off_speed', default_value='0.12')
    back_off_growth_sec_arg = DeclareLaunchArgument('back_off_growth_sec', default_value='0.40')
    back_off_max_time_sec_arg = DeclareLaunchArgument(
        'back_off_max_time_sec', default_value='5.00')
    turn_direction_hysteresis_m_arg = DeclareLaunchArgument(
        'turn_direction_hysteresis_m', default_value='0.10')
    turn_time_sec_arg = DeclareLaunchArgument('turn_time_sec', default_value='2.30')
    turn_speed_arg = DeclareLaunchArgument('turn_speed', default_value='0.30')
    forward_speed_arg = DeclareLaunchArgument('forward_speed', default_value='0.12')
    forward_distance_m_arg = DeclareLaunchArgument('forward_distance_m', default_value='1.90')
    forward_time_sec_arg = DeclareLaunchArgument('forward_time_sec', default_value='15.00')
    yaw_tolerance_deg_arg = DeclareLaunchArgument('yaw_tolerance_deg', default_value='5.0')
    search_turn_speed_arg = DeclareLaunchArgument('search_turn_speed', default_value='0.12')
    line_search_error_threshold_arg = DeclareLaunchArgument(
        'line_search_error_threshold', default_value='20.0')
    line_search_confirm_count_arg = DeclareLaunchArgument(
        'line_search_confirm_count', default_value='8')
    search_max_yaw_deviation_deg_arg = DeclareLaunchArgument(
        'search_max_yaw_deviation_deg', default_value='130.0')
    search_timeout_sec_arg = DeclareLaunchArgument('search_timeout_sec', default_value='45.0')
    publish_rate_hz_arg = DeclareLaunchArgument('publish_rate_hz', default_value='20.0')

    line_topic_arg = DeclareLaunchArgument('line_topic', default_value='/cmd_vel_line')
    avoid_topic_arg = DeclareLaunchArgument('avoid_topic', default_value='/cmd_vel_obstacle')
    output_topic_arg = DeclareLaunchArgument('output_topic', default_value='/cmd_vel')
    rate_hz_arg = DeclareLaunchArgument('rate_hz', default_value='20.0')

    detector = Node(
        package='line_follower',
        executable='line_detector',
        name='line_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'roi_start': LaunchConfiguration('roi_start'),
            'line_is_dark': LaunchConfiguration('line_is_dark'),
            'use_adaptive': LaunchConfiguration('use_adaptive'),
            'adaptive_block': LaunchConfiguration('adaptive_block'),
            'adaptive_c': LaunchConfiguration('adaptive_c'),
            'kernel_size': LaunchConfiguration('kernel_size'),
            'fixed_thresh': LaunchConfiguration('fixed_thresh'),
            'use_hsv': LaunchConfiguration('use_hsv'),
            'hsv_lower_h': LaunchConfiguration('hsv_lower_h'),
            'hsv_lower_s': LaunchConfiguration('hsv_lower_s'),
            'hsv_lower_v': LaunchConfiguration('hsv_lower_v'),
            'hsv_upper_h': LaunchConfiguration('hsv_upper_h'),
            'hsv_upper_s': LaunchConfiguration('hsv_upper_s'),
            'hsv_upper_v': LaunchConfiguration('hsv_upper_v'),
            'min_nonzero': LaunchConfiguration('min_nonzero'),
            'max_fill_ratio': LaunchConfiguration('max_fill_ratio'),
            'min_contour_area': LaunchConfiguration('min_contour_area'),
            'lost_sentinel': LaunchConfiguration('lost_sentinel'),
            'ema_alpha': LaunchConfiguration('ema_alpha'),
            'max_contour_jump': LaunchConfiguration('max_contour_jump'),
            'contour_switch_confirm_frames': LaunchConfiguration('contour_switch_confirm_frames'),
        }],
    )

    controller = Node(
        package='line_follower',
        executable='line_controller',
        name='line_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'linear_x': LaunchConfiguration('linear_x'),
            'k_p': LaunchConfiguration('k_p'),
            'max_ang_z': LaunchConfiguration('max_ang_z'),
            'steer_sign': LaunchConfiguration('steer_sign'),
            'search_w': LaunchConfiguration('search_w'),
            'search_linear_x': LaunchConfiguration('search_linear_x'),
            'lost_sentinel': LaunchConfiguration('lost_sentinel'),
            'min_linear_x': LaunchConfiguration('min_linear_x'),
            'slowdown_error': LaunchConfiguration('slowdown_error'),
            'turn_in_place_error': LaunchConfiguration('turn_in_place_error'),
            'error_deadband': LaunchConfiguration('error_deadband'),
            'angular_alpha': LaunchConfiguration('angular_alpha'),
            'lost_timeout_sec': LaunchConfiguration('lost_timeout_sec'),
        }],
    )

    avoider = Node(
        package='tb3_safety',
        executable='obstacle_avoid',
        name='obstacle_avoid',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': scan_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'front_half_angle_deg': front_half_angle_deg,
            'side_sector_max_deg': side_sector_max_deg,
            'lidar_to_hull_margin': lidar_to_hull_margin,
            'avoid_distance': avoid_distance,
            'emergency_distance': emergency_distance,
            'stop_time_sec': stop_time_sec,
            'back_off_speed': back_off_speed,
            'back_off_growth_sec': back_off_growth_sec,
            'back_off_max_time_sec': back_off_max_time_sec,
            'turn_direction_hysteresis_m': turn_direction_hysteresis_m,
            'turn_time_sec': turn_time_sec,
            'turn_speed': turn_speed,
            'forward_speed': forward_speed,
            'forward_distance_m': forward_distance_m,
            'forward_time_sec': forward_time_sec,
            'yaw_tolerance_deg': yaw_tolerance_deg,
            'search_turn_speed': search_turn_speed,
            'line_search_error_threshold': line_search_error_threshold,
            'line_search_confirm_count': line_search_confirm_count,
            'search_max_yaw_deviation_deg': search_max_yaw_deviation_deg,
            'search_timeout_sec': search_timeout_sec,
            'publish_rate_hz': publish_rate_hz,
        }],
    )

    supervisor = Node(
        package='line_follower',
        executable='supervisor',
        name='supervisor',
        output='screen',
        parameters=[{
            'line_topic': line_topic,
            'avoid_topic': avoid_topic,
            'output_topic': output_topic,
            'rate_hz': rate_hz,
        }],
    )

    return LaunchDescription([
        env_qt,
        use_sim_time_arg,
        roi_start_arg,
        line_is_dark_arg,
        use_adaptive_arg,
        adaptive_block_arg,
        adaptive_c_arg,
        kernel_size_arg,
        fixed_thresh_arg,
        use_hsv_arg,
        hsv_lower_h_arg,
        hsv_lower_s_arg,
        hsv_lower_v_arg,
        hsv_upper_h_arg,
        hsv_upper_s_arg,
        hsv_upper_v_arg,
        min_nonzero_arg,
        max_fill_ratio_arg,
        min_contour_area_arg,
        lost_sentinel_arg,
        ema_alpha_arg,
        max_contour_jump_arg,
        contour_switch_confirm_frames_arg,
        linear_x_arg,
        k_p_arg,
        max_ang_z_arg,
        steer_sign_arg,
        search_w_arg,
        search_linear_x_arg,
        min_linear_x_arg,
        slowdown_error_arg,
        turn_in_place_error_arg,
        error_deadband_arg,
        angular_alpha_arg,
        lost_timeout_sec_arg,
        scan_topic_arg,
        cmd_vel_topic_arg,
        front_half_angle_deg_arg,
        side_sector_max_deg_arg,
        lidar_to_hull_margin_arg,
        avoid_distance_arg,
        emergency_distance_arg,
        stop_time_sec_arg,
        back_off_speed_arg,
        back_off_growth_sec_arg,
        back_off_max_time_sec_arg,
        turn_direction_hysteresis_m_arg,
        turn_time_sec_arg,
        turn_speed_arg,
        forward_speed_arg,
        forward_distance_m_arg,
        forward_time_sec_arg,
        yaw_tolerance_deg_arg,
        search_turn_speed_arg,
        line_search_error_threshold_arg,
        line_search_confirm_count_arg,
        search_max_yaw_deviation_deg_arg,
        search_timeout_sec_arg,
        publish_rate_hz_arg,
        line_topic_arg,
        avoid_topic_arg,
        output_topic_arg,
        rate_hz_arg,
        detector,
        controller,
        avoider,
        supervisor,
    ])
