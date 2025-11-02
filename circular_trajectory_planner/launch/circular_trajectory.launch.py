from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('circular_trajectory_planner')
    config_file = os.path.join(pkg_share, 'config', 'trajectory_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Path to trajectory parameters file'
        ),
        
        Node(
            package='circular_trajectory_planner',
            executable='circular_trajectory_planner_node',
            name='circular_trajectory_planner',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        ),
    ])