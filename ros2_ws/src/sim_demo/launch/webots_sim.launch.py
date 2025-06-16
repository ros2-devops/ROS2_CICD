from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    duration = LaunchConfiguration('duration')
    log_interval = LaunchConfiguration('log_interval')
    scenario = LaunchConfiguration('scenario')

    world_path = os.path.join(
        os.getcwd(), 'ros2_ws', 'src', 'sim_demo', 'worlds',
        LaunchConfiguration('scenario').perform({}) + '.wbt'
    )

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='10'),
        DeclareLaunchArgument('log_interval', default_value='1.0'),
        DeclareLaunchArgument('scenario', default_value='demo1'),

        ExecuteProcess(
            cmd=[
                'webots', '--stdout', '--batch', world_path
            ],
            additional_env={
                'SIM_DURATION': duration,
                'LOG_INTERVAL': log_interval,
                'SCENARIO': scenario
            },
            shell=True
        ),

        ExecuteProcess(
            cmd=['ros2', 'run', 'ros2_observability', 'metrics_collector'],
            shell=True,
            output='screen'
        )
    ])
