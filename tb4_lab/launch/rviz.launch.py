# RViz2 với cấu hình TurtleBot 4 (config/tb4.rviz). fixed_frame: odom khi chưa có map, map khi đã có.
import os
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fixed_frame', default_value='map'),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg('tb4_lab'), 'config', 'tb4.rviz'),
                        '-f', LC('fixed_frame')],
             parameters=[{'use_sim_time': True}],
             additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}),   # RViz render GPU trên WSL2 bị treo khung 3D khi bật costmap
    ])
