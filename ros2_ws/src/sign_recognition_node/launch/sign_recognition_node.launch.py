from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    sign_recognition = Node(
        package='sign_recognition_node',
        executable='sign_recognition_node_main',
        name='sign_recognition_node',
        output='screen',
    )

    return LaunchDescription([
        sign_recognition,
    ])
