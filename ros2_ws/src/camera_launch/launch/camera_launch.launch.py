import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():

    arena_camera_node_launch_dir = os.path.join(get_package_share_directory('arena_camera_node'),
                                                'launch',
                                                'start_arena_camera.launch.py')
    arena_camera_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(arena_camera_node_launch_dir))

    camera_manager_launch_dir = os.path.join(get_package_share_directory('camera_manager'),
                                                'launch',
                                                'camera_manager.launch.py')
    camera_manager = IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_manager_launch_dir))

    return LaunchDescription([
        arena_camera_node,
        camera_manager,
    ])
