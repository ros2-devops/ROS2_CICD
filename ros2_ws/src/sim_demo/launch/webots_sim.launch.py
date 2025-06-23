from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution

def generate_launch_description():
    duration = LaunchConfiguration('duration')
    log_interval = LaunchConfiguration('log_interval')
    scenario = LaunchConfiguration('scenario')

    # Dynamically construct world path using PathJoinSubstitution
    world_path = PathJoinSubstitution([
        FindPackageShare('sim_demo'),
        'worlds',
        scenario,
        TextSubstitution(text='.wbt')
    ])

    return LaunchDescription([
        DeclareLaunchArgument('duration', default_value=TextSubstitution(text='10')),
        DeclareLaunchArgument('log_interval', default_value=TextSubstitution(text='1.0')),
        DeclareLaunchArgument('scenario', default_value=TextSubstitution(text='demo1')),

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
