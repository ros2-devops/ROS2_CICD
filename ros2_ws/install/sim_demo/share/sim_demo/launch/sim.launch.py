from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_demo',
            executable='sim_demo',
            name='sim_node',
            output='screen'
        )
    ])

