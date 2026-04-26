import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('autonomous_robot_localization_pkg')

    # ── Configuration files ──────────────────────────────────────────────────
    imu_config = os.path.join(pkg_share, 'config', 'imu_parameters.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf_parameters.yaml')

    # ── Camera pipeline ──────────────────────────────────────────────────────
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

    # ── IMU ──────────────────────────────────────────────────────────────────
    imu_node = Node(
        package='autonomous_robot_localization_pkg',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[imu_config]
    )

    # ── EKF (sensor fusion) ──────────────────────────────────────────────────
    ekf_node = Node(
        package='autonomous_robot_localization_pkg',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_config]
    )

    # ── Mapper ───────────────────────────────────────────────────────────────
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
        imu_node,
        ekf_node,
        mapper_node,
    ])
