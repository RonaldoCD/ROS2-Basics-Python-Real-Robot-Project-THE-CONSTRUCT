from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower',
            executable='wall_finder',
            output='screen'),
        Node(
            package='wall_follower',
            executable='wall_follower',
            output='screen'),
    ])
