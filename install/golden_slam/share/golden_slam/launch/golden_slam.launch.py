from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('golden_slam'),
        'rviz',
        'golden_slam.rviz'
    )
    print(f"[golden_slam.launch] rviz_config = {rviz_config}")

    slam_node = Node(
        package='golden_slam',
        executable='golden_slam_node',
        name='golden_slam',
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