import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstacle_detection',
            executable='image_detection',
            name='image_detection_node',
            output='screen',
            # Add any additional parameters or remappings here if needed
        ),
        Node(
            package='obstacle_detection',
            executable='can_detection',
            name='can_detection_node',
            output='screen',
            # Add any additional parameters or remappings here if needed
        )
    ])
