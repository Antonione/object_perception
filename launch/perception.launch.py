from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_perception',
            executable='object_detector',
            name='object_detector',
            output='screen'
        ),
        Node(
            package='object_perception',
            executable='static_tf_publisher',
            name='static_tf_publisher',
            output='screen'
        )
    ])
