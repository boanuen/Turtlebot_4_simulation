# Localization (map_server + AMCL từ map đã lưu) + Nav2 + RViz2. AMCL và Nav2 đọc /scan_nav (trễ 0.1 s).
import os
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource as Src
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import SetRemap


def generate_launch_description():
    nav = os.path.join(pkg('turtlebot4_navigation'), 'launch')
    rviz = os.path.join(pkg('tb4_lab'), 'launch', 'rviz.launch.py')
    default_map = os.path.join(  # realpath: về thư mục maps/ trong source khi build --symlink-install
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'maps', 'map.yaml')
    sim_time = {'use_sim_time': 'true'}
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        GroupAction([SetRemap('scan', 'scan_nav'),
                     IncludeLaunchDescription(Src(os.path.join(nav, 'localization.launch.py')),
                                              launch_arguments={**sim_time, 'map': LC('map')}.items()),
                     IncludeLaunchDescription(Src(os.path.join(nav, 'nav2.launch.py')),
                                              launch_arguments=sim_time.items())]),
        IncludeLaunchDescription(Src(rviz)),
    ])
