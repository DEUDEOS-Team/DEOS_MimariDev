#!/usr/bin/env python3
"""Test listener node for debugging mock sensor topics"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
import sys


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.pc_sub = self.create_subscription(PointCloud2, '/points_downsampled', self.pointcloud_callback, 10)
        
        self.get_logger().info("Listener started, waiting for messages...")
        
        self.imu_count = 0
        self.gps_count = 0
        self.pc_count = 0

    def imu_callback(self, msg):
        self.imu_count += 1
        self.get_logger().info(f"[{self.imu_count}] IMU: accel=({msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f})")

    def gps_callback(self, msg):
        self.gps_count += 1
        self.get_logger().info(f"[{self.gps_count}] GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.2f}")

    def pointcloud_callback(self, msg):
        self.pc_count += 1
        self.get_logger().info(f"[{self.pc_count}] PointCloud: {msg.width} points")


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"Received {node.imu_count} IMU, {node.gps_count} GPS, {node.pc_count} PointCloud messages")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
