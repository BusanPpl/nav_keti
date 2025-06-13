import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    current_dir = os.path.dirname(__file__)
    param_file = os.path.join(current_dir, '../config/params.yaml')

    costmap_generator_launch = os.path.join(
        get_package_share_directory('nav_keti'), 
        'launch', 
        'costmap_generator.launch.py'
    )

    sim_launch = os.path.join(
        get_package_share_directory('urdf_keti'), 
        'launch', 
        'sim.launch.py'
    )

    multi_object_launch = os.path.join(
        get_package_share_directory('multiple_object_tracking_lidar'), 
        'launch', 
        'multiple_object_tracking_lidar.launch.py'
    )

    pathplanning_launch = os.path.join(
        get_package_share_directory('nav_keti'), 
        'launch', 
        'path_planning.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(costmap_generator_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi_object_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pathplanning_launch)
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
