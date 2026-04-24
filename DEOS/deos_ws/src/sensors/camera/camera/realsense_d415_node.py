import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class RealSenseD415Node(Node):
    def __init__(self):
        super().__init__('realsense_d415_node')
        
        # Parameters
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_info_topic', '/camera/color/camera_info')
        self.declare_parameter('depth_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30)
        
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        
        self.get_logger().info("Initializing RealSense D415...")
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable RGB and Depth streams
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        
        try:
            # Start pipeline
            profile = self.pipeline.start(self.config)
            self.get_logger().info("RealSense pipeline started successfully")
            
            # Get stream profiles for camera info
            color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
            depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            
            self.color_intrinsics = color_profile.get_intrinsics()
            self.depth_intrinsics = depth_profile.get_intrinsics()
            
            # Align depth to color frame
            self.align = rs.align(rs.stream.color)
            
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense: {e}")
            self.get_logger().error("Make sure RealSense D415 is connected via USB 3.0")
            return
        
        # Publishers
        self.rgb_publisher = self.create_publisher(Image, rgb_topic, 10)
        self.depth_publisher = self.create_publisher(Image, depth_topic, 10)
        self.color_info_publisher = self.create_publisher(CameraInfo, self.get_parameter('color_info_topic').value, 10)
        self.depth_info_publisher = self.create_publisher(CameraInfo, self.get_parameter('depth_info_topic').value, 10)
        
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # Timer for capture loop (at specified FPS)
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.capture_frame)
        
        self.get_logger().info(f"RealSense D415 node ready")
        self.get_logger().info(f"  RGB:   {rgb_topic}")
        self.get_logger().info(f"  Depth: {depth_topic}")
        self.get_logger().info(f"  Resolution: {width}x{height} @ {fps}fps")

    def capture_frame(self):
        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            # Align depth to color
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                self.get_logger().warn("Missing color or depth frame")
                return
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Create Image messages
            timestamp = self.get_clock().now().to_msg()
            
            # RGB message
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = "camera_color_frame"
            
            # Depth message (16-bit depth in millimeters)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
            depth_msg.header.stamp = timestamp
            depth_msg.header.frame_id = "camera_depth_frame"
            
            # Publish messages
            self.rgb_publisher.publish(color_msg)
            self.depth_publisher.publish(depth_msg)
            
            # Publish camera info
            self.color_info_publisher.publish(self.create_camera_info(self.color_intrinsics, timestamp, "camera_color_frame"))
            self.depth_info_publisher.publish(self.create_camera_info(self.depth_intrinsics, timestamp, "camera_depth_frame"))
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                self.get_logger().info(f"Published {self.frame_count} frame pairs")
        
        except Exception as e:
            self.get_logger().error(f"Capture error: {e}")

    def create_camera_info(self, intrinsics, timestamp, frame_id):
        """Create CameraInfo message from RealSense intrinsics"""
        info = CameraInfo()
        info.header.stamp = timestamp
        info.header.frame_id = frame_id
        info.width = intrinsics.width
        info.height = intrinsics.height
        
        # Focal length and principal point
        info.K = [
            intrinsics.fx, 0, intrinsics.ppx,
            0, intrinsics.fy, intrinsics.ppy,
            0, 0, 1
        ]
        
        # Distortion model and coefficients
        info.distortion_model = "plumb_bob"
        info.D = list(intrinsics.coeffs)
        
        return info

    def destroy_node(self):
        self.pipeline.stop()
        self.get_logger().info("RealSense pipeline stopped")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseD415Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down RealSense D415 node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
