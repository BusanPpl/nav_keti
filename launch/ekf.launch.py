import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    param_file = os.path.join(current_dir, '../config/ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[param_file]
    )



    return LaunchDescription([
        ekf_node
    ])
