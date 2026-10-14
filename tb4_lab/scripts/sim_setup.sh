#!/bin/bash
# Chạy sau khi spawn robot. Dùng: sim_setup.sh <world> <rtf>
# 1) Gỡ trạm sạc: nằm sát mũi robot, hiện thành vật cản trên map và làm Nav2 kẹt.
# 2) Giới hạn tốc độ mô phỏng (real time factor): SLAM, RViz và Gazebo tranh CPU, chạy quá nhanh thì SLAM bị trễ và bản đồ lệch.
# 3) safety_override full: tắt reflex bumper/backup-limit của Create 3, nếu không robot tự khóa lệnh /cmd_vel.
has_dock() { timeout 10 ign topic -e -t /world/"$1"/pose/info -n 1 2>/dev/null | grep -q 'name: "standard_dock"'; }
for i in $(seq 60); do has_dock "$1" && break; sleep 2; done   # chờ dock được spawn
for i in 1 2 3 4 5; do
  ign service -s /world/"$1"/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean \
    --timeout 3000 --req 'name: "standard_dock" type: MODEL' >/dev/null 2>&1
  sleep 2; has_dock "$1" || break
done
ign service -s /world/"$1"/set_physics --reqtype ignition.msgs.Physics --reptype ignition.msgs.Boolean \
  --timeout 3000 --req "max_step_size: 0.003 real_time_factor: ${2:-0.5}" >/dev/null 2>&1
for i in $(seq 40); do
  ros2 param set /motion_control safety_override full 2>/dev/null | grep -q successful && exit 0
  sleep 3
done
