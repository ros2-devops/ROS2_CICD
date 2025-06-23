from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution,
)

def generate_launch_description():
    # launch arguments
    duration     = LaunchConfiguration("duration")
    log_interval = LaunchConfiguration("log_interval")
    scenario     = LaunchConfiguration("scenario")

    # world file to load: ros2_ws/src/sim_demo/worlds/<scenario>.wbt
    world_path = PathJoinSubstitution(
        [
            TextSubstitution(text="ros2_ws/src/sim_demo/worlds/"),
            scenario,
            TextSubstitution(text=".wbt"),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("duration", default_value="180"),
            DeclareLaunchArgument("log_interval", default_value="1.0"),
            DeclareLaunchArgument("scenario",  default_value="demo1"),

            # ── Webots head-less -------------------------------------------------
            ExecuteProcess(
                cmd=[
                    "webots",
                    "--stdout",
                    "--batch",
                    world_path,
                ],
                additional_env={
                    "SIM_DURATION": duration,
                    "LOG_INTERVAL": log_interval,
                    "SCENARIO":     scenario,
                },
                output="screen",
            ),

            # ── Metrics collector node -----------------------------------------
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "ros2_observability",
                    "metrics_collector",
                ],
                output="screen",
            ),
        ]
    )
