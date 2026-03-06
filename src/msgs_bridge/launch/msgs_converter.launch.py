from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='msgs_bridge',
            executable='action_converter',
            name='action_converter',
            output='screen'
        ),
        
        Node(
            package='msgs_bridge',
            executable='status_converter',
            name='status_converter',
            output='screen'
        )
    ])