from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_demo',
            executable='sim_demo',
            name='sim_demo_node'
        ),
        Node(
            package='ros2_observability',
            executable='metrics_collector',
            name='metrics_node'
        )
    ])
