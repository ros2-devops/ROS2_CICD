from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    duration = LaunchConfiguration('duration')
    log_interval = LaunchConfiguration('log_interval')
    scenario = LaunchConfiguration('scenario')

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value='180'),
        DeclareLaunchArgument('log_interval', default_value='1.0'),
        DeclareLaunchArgument('scenario', default_value='demo1'),

        # Run Webots in headless mode using shell expansion for world path
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'webots --stdout --batch ros2_ws/src/sim_demo/worlds/$SCENARIO.wbt'
            ],
            additional_env={
                'SCENARIO': scenario,
                'SIM_DURATION': duration,
                'LOG_INTERVAL': log_interval
            },
            shell=True
        ),

        # Start ROS 2 metrics collector node
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros2_observability', 'metrics_collector'],
            shell=True,
            output='screen'
        )
    ])
