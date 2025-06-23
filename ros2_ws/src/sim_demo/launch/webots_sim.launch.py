from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='180'),
        DeclareLaunchArgument('log_interval', default_value='1.0'),
        DeclareLaunchArgument('scenario', default_value='demo1'),

        # Launch Webots simulation with world file based on scenario
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'webots --stdout --batch ros2_ws/src/sim_demo/worlds/${SCENARIO}.wbt'
            ],
            additional_env={
                'SCENARIO': LaunchConfiguration('scenario'),
                'SIM_DURATION': LaunchConfiguration('duration'),
                'LOG_INTERVAL': LaunchConfiguration('log_interval'),
            },
            shell=True
        ),

        # Launch the metrics collector node
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros2_observability', 'metrics_collector'],
            shell=True,
            output='screen'
        )
    ])
