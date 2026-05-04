import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import serial
import pynmea2
from datetime import datetime


class GPSNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('frame_id', 'gps_link')
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Serial setup
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"GPS connected on {port} @ {baudrate} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to open GPS port: {e}")
            self.ser = None
            return
        
        # Publisher
        self.publisher_ = self.create_publisher(NavSatFix, '/gps/fix', 10)
        
        # Timer for read loop
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("GPS node started")

    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            return
        
        try:
            line = self.ser.readline()
            if not line:
                return
            
            line = line.decode('utf-8', errors='ignore').strip()
            if not line or not line.startswith('$'):
                return
            
            # Parse NMEA sentence
            try:
                msg = pynmea2.parse(line)
            except:
                return
            
            # Only process GGA (position fix) and RMC (recommended) sentences
            if not isinstance(msg, (pynmea2.GGA, pynmea2.RMC)):
                return
            
            # Check fix quality
            if hasattr(msg, 'fix_stat') and msg.fix_stat == '0':
                self.get_logger().warn("GPS: No fix")
                return
            
            if hasattr(msg, 'gps_qual') and msg.gps_qual == '0':
                self.get_logger().warn("GPS: Invalid data")
                return
            
            # Extract latitude and longitude
            if hasattr(msg, 'lat') and hasattr(msg, 'lon') and msg.lat and msg.lon:
                lat = float(msg.lat)
                lon = float(msg.lon)
                
                # Altitude (if available)
                alt = 0.0
                if hasattr(msg, 'altitude') and msg.altitude:
                    alt = float(msg.altitude)
                
                # Create NavSatFix message
                nav_msg = NavSatFix()
                nav_msg.header.stamp = self.get_clock().now().to_msg()
                nav_msg.header.frame_id = self.frame_id
                nav_msg.latitude = lat
                nav_msg.longitude = lon
                nav_msg.altitude = alt
                nav_msg.status.status = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                nav_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
                
                self.publisher_.publish(nav_msg)
                self.get_logger().debug(f"GPS: Lat={lat}, Lon={lon}, Alt={alt}")
        
        except Exception as e:
            self.get_logger().error(f"GPS parsing error: {e}")

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
