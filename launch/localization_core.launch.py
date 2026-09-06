import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE_NAME = 'autonomous_robot_localization_pkg'


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    core_params = os.path.join(
        package_share,
        'config',
        'localization_core.yaml',
    )

    visual_node = Node(
        package=PACKAGE_NAME,
        executable='visual_node',
        name='visual_node',
        output='screen',
        parameters=[core_params],
    )

    ekf_node = Node(
        package=PACKAGE_NAME,
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[core_params],
    )

    mapper_node = Node(
        package=PACKAGE_NAME,
        executable='mapper_node',
        name='mapper_node',
        output='screen',
        parameters=[core_params],
    )

    streaming_node = Node(
        package=PACKAGE_NAME,
        executable='streaming_node',
        name='streaming_node',
        output='screen',
        parameters=[core_params],
    )

    return LaunchDescription([
        visual_node,
        ekf_node,
        mapper_node,
        streaming_node,
    ])
