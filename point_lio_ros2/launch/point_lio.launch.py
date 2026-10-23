import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    root = os.environ.get('POINT_LIO_ROOT', os.path.join(os.path.expanduser('~'), 'Point-LIO'))
    config = os.environ.get('POINT_LIO_CONFIG', os.path.join(root, 'config', 'mid360.yaml'))
    save_path = os.environ.get('POINT_LIO_MAP_DIR', os.path.join(os.path.expanduser('~'), 'point_lio_maps'))
    return LaunchDescription([
        Node(
            package='point_lio_ros2',
            executable='point_lio_node',
            name='point_lio',
            output='screen',
            parameters=[{
                'config_file': config,
                'map_save_path': save_path,
            }],
        ),
    ])
