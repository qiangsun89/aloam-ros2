import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('a_loam_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'aloam_params.yaml')

    scan_registration_node = Node(
        package='a_loam_ros2',
        executable='scan_registration_node',
        name='scan_registration_node',
        parameters=[config_file, {'scan_line': 64}],
        output='screen'
    )

    laser_odometry_node = Node(
        package='a_loam_ros2',
        executable='laser_odometry_node',
        name='laser_odometry_node',
        parameters=[config_file],
        output='screen'
    )

    laser_mapping_node = Node(
        package='a_loam_ros2',
        executable='laser_mapping_node',
        name='laser_mapping_node',
        parameters=[config_file],
        output='screen'
    )

    return LaunchDescription([
        scan_registration_node,
        laser_odometry_node,
        laser_mapping_node,
    ])
