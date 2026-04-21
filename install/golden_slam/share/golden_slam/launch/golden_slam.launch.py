from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
    return LaunchDescription([
        Node(
            package='golden_slam',
            executable='golden_slam_node',
            name='golden_slam',
            output='screen'
        )
    ])