import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    param_file = os.path.join(current_dir, '../config/params.yaml')

    return LaunchDescription([

        Node(
            package='nav_keti',
            executable='local_path',
            name='local_path',
            parameters=[param_file],
            output='screen'
        )
    ])