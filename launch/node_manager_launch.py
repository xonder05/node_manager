from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    config_file_path = PathJoinSubstitution([
        FindPackageShare("node_manager"),
        "config",
        "_manager.yaml"
    ])
    
    node_manager = Node(
        package="node_manager",
        executable="node_manager",
        parameters=[config_file_path],
        output="screen",
    )
    
    return LaunchDescription([
        node_manager,
    ])
