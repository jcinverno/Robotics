import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_goal',
            executable='nav_goal',
            name='nav_goal_node',
            output='screen',
            # Add any additional parameters or remappings here if needed
        )
    ])
