import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    ekf_param_file = os.path.join(current_dir, '../config/ekf.yaml')
    navsat_param_file = os.path.join(current_dir, '../config/navsat.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{'use_sim_time': False}, ekf_param_file]
    )

    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[{'use_sim_time': False}, navsat_param_file]
    )

    return LaunchDescription([
        ekf_node,
        navsat_node
    ])
