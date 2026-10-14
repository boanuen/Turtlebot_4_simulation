#!/usr/bin/env python3
# /scan -> /scan_nav, trễ 0.1 s (giữ nguyên header.stamp).
# Trên Humble, nếu scan đến trước TF tương ứng thì tf2_ros::MessageFilter của SLAM Toolbox, AMCL và costmap
# có thể bị treo (map -> odom ngừng cập nhật). Trễ 0.1 s đảm bảo TF đã có khi scan tới nơi.
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DELAY = 0.1


class ScanDelay(Node):
    def __init__(self):
        super().__init__('scan_delay')
        self.q = deque()
        self.pub = self.create_publisher(LaserScan, 'scan_nav', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(LaserScan, 'scan', lambda m: self.q.append((self.get_clock().now(), m)), qos_profile_sensor_data)
        self.create_timer(0.02, self.flush)

    def flush(self):
        now = self.get_clock().now()
        while self.q and (now - self.q[0][0]).nanoseconds * 1e-9 >= DELAY:
            self.pub.publish(self.q.popleft()[1])


def main():
    rclpy.init()
    rclpy.spin(ScanDelay())


if __name__ == '__main__':
    main()
