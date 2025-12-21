from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    photographer_node = Node(
        package='location_pkg',
        executable='photographer_node',
        name='photographer_node',
        output='screen'
    )

    visual_node = Node(
        package='location_pkg',
        executable='visual_node',
        name='visual_node',
        output='screen'
    )

    return LaunchDescription([
        photographer_node,
        visual_node
    ])
