from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description(): 
    pkg_share = get_package_share_directory('golden_slam')
    rviz_config = os.path.join(pkg_share, 'rviz', 'golden_slam.rviz')

    return LaunchDescription([
        Node(
            package='golden_slam',
            executable='golden_slam_node',
            name='golden_slam',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])