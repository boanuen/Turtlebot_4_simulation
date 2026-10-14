# SLAM Toolbox (online async) + RViz2. Publish /map và TF map->odom.
# scan_topic:=scan_nav để đọc /scan trễ 0.1 s (tránh treo MessageFilter của tf2_ros trên Humble, xem scripts/scan_delay.py).
import os
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource as Src
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import SetRemap


def generate_launch_description():
    slam = os.path.join(pkg('turtlebot4_navigation'), 'launch', 'slam.launch.py')
    rviz = os.path.join(pkg('tb4_lab'), 'launch', 'rviz.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='scan'),
        GroupAction([SetRemap('scan', LC('scan_topic')),
                     IncludeLaunchDescription(Src(slam), launch_arguments={'use_sim_time': 'true', 'sync': 'false'}.items())]),
        IncludeLaunchDescription(Src(rviz)),
    ])
