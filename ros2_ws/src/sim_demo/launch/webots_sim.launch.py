from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    duration = LaunchConfiguration('duration')
    log_interval = LaunchConfiguration('log_interval')

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='10'),
        DeclareLaunchArgument('log_interval', default_value='1.0'),

        Node(
            package='ros2_observability',
            executable='metrics_collector',
            name='metrics_node',
            output='screen',
            parameters=[{
                'duration': duration,
                'log_interval': log_interval
            }],
            # these will become environment variables if needed:
            additional_env={'SIM_DURATION': duration, 'LOG_INTERVAL': log_interval}
        ),

        Node(
            package='sim_demo',
            executable='simulator_node',
            name='sim_demo',
            output='screen'
        ),
    ])
