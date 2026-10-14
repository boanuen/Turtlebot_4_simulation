#!/bin/bash
# Tắt toàn bộ mô phỏng, ROS và các script thử còn sót
pkill -9 -f "^/opt/ros/humble/lib"
pkill -9 -f "^/usr/bin/python3 /opt/ros/humble"
pkill -9 -f "^ign gazebo"
pkill -9 -f "^ruby /usr/bin/ign"
pkill -9 -f "[r]os2 launch"
pkill -9 -f "[r]obot_state_publisher"
pkill -9 -f "[s]tatic_transform_publisher"
pkill -9 -f "[s]end_goals.py"
pkill -9 -f "TurtleBot-4-SLAM-Toolbox-Nav2-main"
pkill -9 -f "[r]viz2"
pkill -9 -f "[f]ollow_route.py"
pkill -9 -f "[s]can_delay.py"
pkill -9 -f "[n]av_test.py"
pkill -9 -f "tools/stage[0-9].sh"
sleep 2
