import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    param_file = os.path.join(current_dir, '../config/params.yaml')

    return LaunchDescription([
        # use_sim_time 인자를 선언해줍니다 (기본값 True)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='nav_keti',
            executable='costmap_tracking',
            namespace='costmap_ns',
            name='costmap_tracking',
            parameters=[param_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output='screen'
        )
    ])
