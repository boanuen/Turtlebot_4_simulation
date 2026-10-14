# TurtleBot 4 Simulation: SLAM Toolbox, AMCL và Nav2 (ROS 2 Humble)

Bài thực hành 04, môn Tương tác người - Robot (RBE3053). Mô phỏng TurtleBot 4 trong Ignition Gazebo Fortress,
dựng bản đồ bằng SLAM Toolbox, lưu bản đồ, định vị lại bằng AMCL và dẫn đường tự động bằng Nav2, quan sát trên RViz2.

Package `tb4_lab` không viết lại simulator hay bộ điều khiển. Các launch chỉ gọi lại launch chính thức của
`turtlebot4_ignition_bringup`, `turtlebot4_navigation`, kèm vài cấu hình và script để chạy được trên WSL2.

## Cấu trúc
```
tb4_lab/                  package ROS 2
  launch/sim.launch.py    Gazebo (server và giao diện chạy riêng), TurtleBot 4, scan_delay, sim_setup.sh
  launch/slam.launch.py   SLAM Toolbox + RViz2
  launch/nav.launch.py    map_server + AMCL + Nav2 + RViz2
  launch/rviz.launch.py   RViz2 (tham số fixed_frame)
  config/                 gui.config (Gazebo), tb4.rviz (RViz2)
  maps/                   map.yaml, map.pgm
  scripts/                save_map.sh, sim_setup.sh, xacro, scan_delay.py
tools/                    công cụ đo kiểm (so bản đồ với maze.sdf, đo AMCL và Nav2, script lái thay teleop)
```

## Cài đặt (Ubuntu 22.04 hoặc WSL2)
```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop \
  ros-humble-turtlebot4-navigation ros-humble-teleop-twist-keyboard
mkdir -p ~/ws/src && cd ~/ws/src && git clone https://github.com/boanuen/Turtlebot_4_simulation.git
cd ~/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
source ~/ws/install/setup.bash
```

## Chạy (mỗi lệnh một terminal)
| Bước | Lệnh |
|---|---|
| Phần 1: mô phỏng | `ros2 launch tb4_lab sim.launch.py` (thêm `gui:=false` nếu không cần cửa sổ Gazebo) |
| Kiểm tra topic | `ros2 topic hz /scan`, `ros2 topic echo /odom --once`, `ros2 topic info /cmd_vel` |
| Teleop | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.3` |
| RViz2 (chưa có map) | `ros2 launch tb4_lab rviz.launch.py fixed_frame:=odom` |
| Phần 2: SLAM | `ros2 launch tb4_lab slam.launch.py` |
| Lưu bản đồ | `ros2 run tb4_lab save_map.sh` (ghi vào `maps/`) |
| Phần 3-4: localization và Nav2 | `ros2 launch tb4_lab nav.launch.py` (hoặc `map:=/đường/dẫn/map.yaml`) |

Thứ tự đúng để localization khớp bản đồ: khởi động mô phỏng, chưa di chuyển robot, chạy SLAM rồi mới lái.
Khi localization, chạy lại `sim.launch.py` để robot về điểm xuất phát (cũng là gốc bản đồ), sau đó dùng
2D Pose Estimate trong RViz2 đặt pose tại gốc bản đồ, rồi dùng Nav2 Goal đặt các điểm đến.
Khi quay video, lái robot chậm khi dựng bản đồ (tốc độ quay thấp) để scan matching của SLAM theo kịp.

## Các điều chỉnh để chạy trên WSL2
- Server Gazebo render bằng CPU (render GPU làm LiDAR trả toàn `range_min`); giao diện Gazebo dùng engine `ogre`.
- `scripts/xacro`: chỉ giữ RPLIDAR 10 Hz, tầm đo 5 m; bỏ cảm biến cliff và IR; camera 1 Hz (chỉ có hiệu lực trong `sim.launch.py`).
- `scripts/sim_setup.sh`: gỡ trạm sạc, giới hạn tốc độ mô phỏng (tham số `rtf`, mặc định 0,5), đặt `safety_override: full`.
- `scripts/scan_delay.py`: tạo `/scan_nav` (trễ 0,1 s) cho AMCL và Nav2, tránh treo `tf2_ros::MessageFilter` trên Humble.
  SLAM mặc định đọc `/scan`; có thể thử `slam.launch.py scan_topic:=scan_nav`.
- RViz2 render bằng CPU vì render GPU làm khung 3D đứng hình khi bật costmap.
- Nên cấp thêm CPU cho WSL2 trong `C:\Users\<tên>\.wslconfig` (ví dụ `processors=8`, `memory=10GB`) rồi `wsl --shutdown`.

## Kết quả đã đo
- Bản đồ: 100% ô bị chiếm nằm trong 10 cm so với tường thật (`maze.sdf`), phủ 66,2% vùng tự do.
  SLAM Toolbox ngừng cập nhật sau khoảng 380 s mô phỏng ở lần dựng này nên phần góc trên bên phải chưa quét.
- AMCL: sai số 0,040 m và 0,61 độ khi robot đứng yên.
- Nav2: 3 goal ở 3 hướng đều SUCCEEDED, không phải phục hồi, cách tường tối thiểu 0,38 m.
