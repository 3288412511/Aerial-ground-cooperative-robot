from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_broadcaster1',
            arguments=['0.417', '0', '0', '0', '0', '0', 'base_link', 'scan1'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_broadcaster2',
            arguments=['-0.417', '0', '0', '3.141596', '0', '0', 'base_link', 'scan2'],
        ),
    ])
