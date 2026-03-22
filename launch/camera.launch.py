from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    photographer_node = Node(
        package='autonomous_robot_localization_pkg',
        executable='photographer_node',
        name='photographer_node',
        output='screen'
    )

    visual_node = Node(
        package='autonomous_robot_localization_pkg',
        executable='visual_node',
        name='visual_node',
        output='screen'
    )

    streaming_node = Node(
        package='autonomous_robot_localization_pkg',
        executable='streaming_node',
        name='streaming_node',
        output='screen'
    )

    mapper_node = Node(
        package='autonomous_robot_localization_pkg',
        executable='mapper_node',
        name='mapper_node',
        output='screen'
    )

    return LaunchDescription([
        photographer_node,
        visual_node,
        streaming_node,
        mapper_node
    ])
