from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    ladrc_pkg = get_package_share_directory('ladrc_controller')
    planner_pkg = get_package_share_directory('circular_trajectory_planner')
    
    # Configuration files
    ladrc_config = os.path.join(ladrc_pkg, 'config', 'ladrc_params.yaml')
    planner_config = os.path.join(planner_pkg, 'config', 'trajectory_params.yaml')
    
    return LaunchDescription([
        # LADRC Controller
        Node(
            package='ladrc_controller',
            executable='ladrc_position_controller_node',
            name='ladrc_position_controller',
            parameters=[ladrc_config],
            output='screen'
        ),
        
        # Circular Trajectory Planner
        Node(
            package='circular_trajectory_planner',
            executable='circular_trajectory_planner_node',
            name='circular_trajectory_planner',
            parameters=[planner_config],
            output='screen'
        ),
    ])