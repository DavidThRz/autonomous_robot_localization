import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


PACKAGE_NAME = 'autonomous_robot_localization_pkg'


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    core_launch = os.path.join(package_share, 'launch', 'localization_core.launch.py')
    core_params = os.path.join(
        package_share,
        'config',
        'localization_core.yaml',
    )

    photographer_node = Node(
        package=PACKAGE_NAME,
        executable='photographer_node',
        name='photographer_node',
        output='screen',
        parameters=[core_params],
    )

    imu_node = Node(
        package=PACKAGE_NAME,
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[core_params],
    )

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_launch),
    )

    return LaunchDescription([
        photographer_node,
        imu_node,
        core,
    ])
