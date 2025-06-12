from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    scenario = LaunchConfiguration('scenario')
    duration = LaunchConfiguration('duration')
    log_interval = LaunchConfiguration('log_interval')

    world_path = [os.path.join(
        os.getenv('HOME'),
        'ros2_ws', 'src', 'sim_demo', 'worlds'
    ), '/', scenario, '.wbt']

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='demo1'),
        DeclareLaunchArgument('duration', default_value='10'),
        DeclareLaunchArgument('log_interval', default_value='1.0'),

        ExecuteProcess(
            cmd=[
                'webots', '--stdout', '--batch',
                ''.join(world_path)
            ],
            additional_env={
                'SIM_DURATION': duration,
                'LOG_INTERVAL': log_interval
            },
            shell=True
        ),

        # Start metrics node
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros2_observability', 'metrics_collector'],
            shell=True,
            output='screen'
        )
    ])
