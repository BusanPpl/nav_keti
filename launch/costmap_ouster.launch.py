import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    param_file = os.path.join(current_dir, '../config/params.yaml')

    return LaunchDescription([
        Node(
            package='nav_keti',
            executable='costmap_ouster',
            name='costmap_ouster',
            parameters=[param_file],

        )
    ])