
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node






def generate_launch_description():
    package_path = get_package_share_directory('golden_slam')

    rviz_config = os.path.join(package_path, 'rviz','golden_slam.rviz')

    mid360_config = os.path.join(package_path, 'config','mid360.yaml')

    slam_node = Node(
        package='golden_slam',
        executable='golden_slam_node',
        name='golden_slam',
        parameters=[ mid360_config ],
        output='screen',
        output_format='{line}',
        emulate_tty=True,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        slam_node,
        rviz_node,
    ])