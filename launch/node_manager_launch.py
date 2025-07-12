import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config_file = os.path.join(
        get_package_share_directory('node_manager'),
        'config',
        'config.yaml'
    )

    node_manager = Node(
        package='node_manager',
        executable='agent',
        parameters=[config_file],
        output='screen',
    )
    
    return launch.LaunchDescription([
        node_manager,
    ])