import launch
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='answer',
            executable='img_handle',
            name='img_handle',
            output='screen',
        ),
        Node(
            package='answer',
            executable='Algorithm',
            name='Algorithm',
            output='screen',
        )
    ])