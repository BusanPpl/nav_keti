import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    param_file = os.path.join(current_dir, '../config/params.yaml')
    local_param_file = os.path.join(current_dir, '../config/local_planner_param.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        Node(
            package='nav_keti',
            executable='global_path',
            name='global_path',
            parameters=[param_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
            output='screen'
        ),
        
        # Node(
        #     package='nav_keti',
        #     executable='local_path',
        #     name='local_path',
        #     parameters=[param_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        #     output='screen'
        # )
    ])
