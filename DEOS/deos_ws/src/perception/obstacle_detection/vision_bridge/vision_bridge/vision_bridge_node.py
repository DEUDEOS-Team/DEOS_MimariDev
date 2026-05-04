import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct


class LaneToWallNode(Node):
    def __init__(self):
        super().__init__('vision_bridge_node')
        
        # Parameters
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/lane_walls')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_distance', 5.0)
        
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        self.get_logger().info(f"Initializing vision_bridge_node")
        self.get_logger().info(f"  Input:  {input_topic}")
        self.get_logger().info(f"  Output: {output_topic}")
        
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        self.publisher_ = self.create_publisher(
            PointCloud2,
            output_topic,
            10
        )
        
        self.bridge = CvBridge()
        self.frame_count = 0
        self.get_logger().info("Vision bridge node ready - waiting for camera frames...")

    def image_callback(self, msg):
        self.frame_count += 1
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        points_3d = self.detect_lanes_and_convert(frame)
        
        if points_3d:
            self.publish_point_cloud(points_3d)
            
        if self.frame_count % 30 == 0:
            self.get_logger().info(f"Processed {self.frame_count} frames")

    def detect_lanes_and_convert(self, frame):
        """
        Detect lane markings and convert to 3D points (wall representation).
        This is a simplified version using edge detection.
        Replace with your actual lane detection model.
        """
        points_3d = []
        img_h, img_w = frame.shape[:2]
        
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Edge detection
            edges = cv2.Canny(gray, 50, 150)
            
            # Find contours (lane markings)
            contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Extract points from contours
            for contour in contours:
                if cv2.contourArea(contour) < 50:  # Filter small contours
                    continue
                
                for point in contour:
                    u = int(point[0][0])  # X pixel coordinate
                    v = int(point[0][1])  # Y pixel coordinate
                    
                    # Simple perspective mapping (camera-to-world conversion)
                    # Adjust these calibration values based on your camera setup
                    
                    # Distance in front (Z in world, based on image row)
                    rel_x = (img_h - v) * 0.015
                    
                    # Lateral distance (Y in world, based on image column)
                    rel_y = (img_w / 2.0 - u) * 0.005
                    
                    # Skip points outside valid range
                    if rel_x < 0.2 or rel_x > 5.0:
                        continue
                    
                    # Create vertical wall from ground to 1.5m height
                    for z in [0.0, 0.5, 1.0, 1.5]:
                        points_3d.append([rel_x, rel_y, z])
            
            return points_3d
        
        except Exception as e:
            self.get_logger().error(f"Lane detection error: {e}")
            return []

    def publish_point_cloud(self, points):
        """Convert 3D points to PointCloud2 message and publish."""
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height = 1
        msg.width = len(points)
        
        # Define fields (X, Y, Z coordinates)
        msg.fields = [
            PointField(name='x', offset=0, datatype=7, count=1),   # datatype 7 = float32
            PointField(name='y', offset=4, datatype=7, count=1),
            PointField(name='z', offset=8, datatype=7, count=1)
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        
        # Pack points into binary data
        buffer = []
        for p in points:
            buffer.append(struct.pack('fff', float(p[0]), float(p[1]), float(p[2])))
        
        msg.data = b''.join(buffer)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneToWallNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down vision_bridge_node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
