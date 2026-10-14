#!/bin/bash
# Lưu /map hiện tại ra map.yaml + map.pgm. Dùng: save_map.sh [thư_mục_đích] (mặc định: maps/ của package)
OUT=${1:-$(dirname "$(realpath "$0")")/../maps}
mkdir -p "$OUT"
ros2 run nav2_map_server map_saver_cli -f "$OUT/map" --ros-args -p save_map_timeout:=10.0
