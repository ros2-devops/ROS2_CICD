from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution,
)

def generate_launch_description():
    duration     = LaunchConfiguration("duration")
    log_interval = LaunchConfiguration("log_interval")
    scenario     = LaunchConfiguration("scenario")

    # Build the path to the .wbt file
    # world_path = PathJoinSubstitution([
    #     TextSubstitution(text="ros2_ws/src/sim_demo/worlds/"),
    #     scenario,
    #     TextSubstitution(text=".wbt"),
    # ])
    from launch.substitutions import PythonExpression

    world_path = PythonExpression([
        "'ros2_ws/src/sim_demo/worlds/' + '", scenario, "' + '.wbt'"
    ])

    return LaunchDescription([
        # Launch args
        DeclareLaunchArgument("duration", default_value="180"),
        DeclareLaunchArgument("log_interval", default_value="1.0"),
        DeclareLaunchArgument("scenario", default_value="demo1"),

        # Webots (always headless)
        ExecuteProcess(
            cmd=["webots", "--stdout", "--batch", "--no-rendering", world_path],
            additional_env={
                "SIM_DURATION": duration,
                "LOG_INTERVAL": log_interval,
                "SCENARIO": scenario,
            },
            output="screen",
        ),

        # Metrics collector
        ExecuteProcess(
            cmd=["ros2", "run", "ros2_observability", "metrics_collector"],
            output="screen",
        ),
    ])
