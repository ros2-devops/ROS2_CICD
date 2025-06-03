from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            parameters=[]
        )
    ])
