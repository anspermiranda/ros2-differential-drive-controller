from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='digital_enforcer_pkg',
            executable='digital_enforcer_node',
            name='digital_enforcer_node',
            output='screen'
        )
    ])
