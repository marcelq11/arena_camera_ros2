from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('arena_camera_node'),
        'config',
        'arena_camera_params.yaml'
    )
    camera_node = Node(
            package='arena_camera_node',
            executable='start',
            name='arena_camera_node',
            parameters=[config]
        )

    return LaunchDescription([
        camera_node,
    ])