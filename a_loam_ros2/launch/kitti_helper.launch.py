import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('a_loam_ros2')
    config_file = os.path.join(pkg_dir, 'config', 'aloam_params.yaml')

    kitti_helper_node = Node(
        package='a_loam_ros2',
        executable='kitti_helper_node',
        name='kitti_helper_node',
        parameters=[{
            'dataset_folder': LaunchConfiguration('dataset_folder', default=''),
            'sequence_number': LaunchConfiguration('sequence_number', default='00'),
            'publish_delay': LaunchConfiguration('publish_delay', default=1),
            'to_bag': LaunchConfiguration('to_bag', default='false'),
            'output_bag_file': LaunchConfiguration('output_bag_file', default=''),
        }],
        output='screen'
    )

    # scan_registration_node = Node(
    #     package='a_loam_ros2',
    #     executable='scan_registration_node',
    #     name='scan_registration_node',
    #     parameters=[config_file],
    #     output='screen'
    # )

    # laser_odometry_node = Node(
    #     package='a_loam_ros2',
    #     executable='laser_odometry_node',
    #     name='laser_odometry_node',
    #     parameters=[config_file],
    #     output='screen'
    # )

    # laser_mapping_node = Node(
    #     package='a_loam_ros2',
    #     executable='laser_mapping_node',
    #     name='laser_mapping_node',
    #     parameters=[config_file],
    #     output='screen'
    # )

    return LaunchDescription([
        kitti_helper_node,
        # scan_registration_node,
        # laser_odometry_node,
        # laser_mapping_node,
    ])
