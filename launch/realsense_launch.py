from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for RealSense cameras"""
    
    # Get the package share directory
    pkg_dir = get_package_share_directory('realsense_ros2')
    
    # Declare launch arguments
    config_dir = LaunchConfiguration('config_dir', default=os.path.join(pkg_dir, 'config'))
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(DeclareLaunchArgument(
        'config_dir',
        default_value=config_dir,
        description='Directory for configuration files'
    ))
    
    # Add RealSense nodes
    ld.add_action(Node(
        package='realsense_ros2',
        executable='realsense_node.py',
        name='realsense_node',
        output='screen',
        parameters=[{
            'config_dir': config_dir
        }]
    ))
    
    # Add RQT node for visualization
    ld.add_action(Node(
        package='rqt_graph',
        executable='rqt_graph',
        name='rqt_graph',
        output='screen'
    ))
    
    return ld