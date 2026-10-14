# Ignition Gazebo + TurtleBot 4 (dựa trên ignition.launch.py + turtlebot4_spawn.launch.py gốc).
# Server (vật lý, cảm biến) và GUI chạy tách rời:
#  - server render phần mềm (LIBGL_ALWAYS_SOFTWARE): trên WSL2, render GPU làm LiDAR trả toàn range_min.
#  - GUI render bằng GPU NVIDIA, engine ogre (ogre2 crash trên WSL2); GUI chết thì sim vẫn sống.
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration as LC
from launch_ros.actions import Node


def generate_launch_description():
    bringup = pkg('turtlebot4_ignition_bringup')
    create_bringup = pkg('irobot_create_ignition_bringup')
    paths = [os.path.join(bringup, 'worlds'), os.path.join(create_bringup, 'worlds'),
             str(Path(pkg('turtlebot4_description')).parent.resolve()),
             str(Path(pkg('irobot_create_description')).parent.resolve())]
    gui_plugins = [os.path.join(pkg('turtlebot4_ignition_gui_plugins'), 'lib'),
                   os.path.join(pkg('irobot_create_ignition_plugins'), 'lib')]

    server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pkg('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': [LC('world'), '.sdf -r -s -v 2 ', LC('srv_args')]}.items())

    # GUI riêng: dùng ogre (1) thay vì ogre2 vì ogre2 crash trên WSL2 (GL3PlusTextureGpu::copyTo)
    gui = ExecuteProcess(
        cmd=['env', '-u', 'LIBGL_ALWAYS_SOFTWARE', 'MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA',
             'ign', 'gazebo', '-g', '--render-engine-gui', LC('gui_engine'),
             '--gui-config', os.path.join(pkg('tb4_lab'), 'config', 'gui.config')],
        condition=IfCondition(LC('gui')), output='screen',
        respawn=True, respawn_delay=2.0)  # driver GPU của WSL2 thỉnh thoảng segfault lúc mở GUI

    setup = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('tb4_lab'), 'lib', 'tb4_lab', 'sim_setup.sh'), LC('world'), LC('rtf')])

    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'])

    scan_delay = Node(package='tb4_lab', executable='scan_delay.py', name='scan_delay',
                      parameters=[{'use_sim_time': True}])   # /scan -> /scan_nav (trễ 0.1 s), xem scripts/scan_delay.py

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup, 'launch', 'turtlebot4_spawn.launch.py')),
        launch_arguments={'namespace': '', 'rviz': 'false', 'model': LC('model'), 'x': LC('x'), 'y': LC('y'),
                          'z': '0.0', 'yaw': LC('yaw')}.items())

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='maze'),  # maze | warehouse | depot
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('srv_args', default_value=''),  # tham số thêm cho ign gazebo server
        DeclareLaunchArgument('model', default_value='lite'),  # lite: không có camera OAK-D, nhẹ hơn ~2.5 lần
        DeclareLaunchArgument('gui_engine', default_value='ogre'),
        DeclareLaunchArgument('rtf', default_value='0.5'),  # giới hạn tốc độ mô phỏng so với thời gian thật
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('LP_NUM_THREADS', '2'),  # render CPU chỉ 2 luồng, chừa CPU cho Nav2/RViz
        # scripts/xacro đứng trước xacro thật trong PATH (chỉ trong launch này) để hạ tần số cảm biến
        SetEnvironmentVariable('PATH', [os.path.join(get_package_prefix('tb4_lab'), 'lib', 'tb4_lab'),
                                        ':', EnvironmentVariable('PATH')]),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join(paths)),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', ':'.join(gui_plugins)),
        server, gui, clock, scan_delay, spawn, setup,
    ])
