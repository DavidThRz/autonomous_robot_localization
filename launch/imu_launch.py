import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('autonomous_robot_localization_pkg'),
        'config',
        'imu_parameters.yaml'
    )

    imu_node = Node(
        package='autonomous_robot_localization_pkg',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        imu_node
    ])