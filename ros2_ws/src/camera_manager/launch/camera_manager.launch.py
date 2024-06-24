from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    camera_setting = Node(
        package='camera_manager',
        executable='camera_setting',
        name='camera_setting'
    )
    return LaunchDescription([
        camera_setting,
    ])
