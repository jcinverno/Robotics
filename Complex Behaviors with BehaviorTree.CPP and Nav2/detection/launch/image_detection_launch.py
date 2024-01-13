import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detection',
            executable='image_detection',
            name='image_detect_node',
            output='screen',
            ),
    ])