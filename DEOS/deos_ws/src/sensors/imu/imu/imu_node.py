import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct
from math import pi


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_type', 'mpu9250')  # 'mpu9250', 'vectornav', 'xsens'
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.imu_type = self.get_parameter('imu_type').value
        
        # Serial setup
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"IMU connected on {port} @ {baudrate} baud (type: {self.imu_type})")
        except Exception as e:
            self.get_logger().error(f"Failed to open IMU port: {e}")
            self.ser = None
            return
        
        # Publisher
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        
        # Timer for read loop
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.get_logger().info("IMU node started")

    def timer_callback(self):
        if self.ser is None or not self.ser.is_open:
            return
        
        try:
            line = self.ser.readline()
            if not line:
                return
            
            line = line.decode('utf-8', errors='ignore').strip()
            if not line:
                return
            
            # Parse based on IMU type
            if self.imu_type == 'mpu9250':
                accel, gyro, mag = self.parse_mpu9250(line)
            elif self.imu_type == 'vectornav':
                accel, gyro, mag = self.parse_vectornav(line)
            else:
                return
            
            if accel is None or gyro is None:
                return
            
            # Create Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id
            
            # Accelerometer (m/s²)
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]
            
            # Gyroscope (rad/s)
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]
            
            # Covariance (optional, set reasonable values)
            imu_msg.linear_acceleration_covariance = [0.01] * 9
            imu_msg.angular_velocity_covariance = [0.01] * 9
            imu_msg.orientation_covariance = [-1] * 9  # -1 means orientation not available
            
            self.publisher_.publish(imu_msg)
        
        except Exception as e:
            self.get_logger().error(f"IMU parsing error: {e}")

    def parse_mpu9250(self, line):
        """Parse MPU9250 format: ax,ay,az,gx,gy,gz"""
        try:
            parts = line.split(',')
            if len(parts) < 6:
                return None, None, None
            
            accel = [float(parts[i]) * 9.81 / 16384.0 for i in range(3)]  # Convert to m/s²
            gyro = [float(parts[i]) * pi / 180.0 / 131.0 for i in range(3, 6)]  # Convert to rad/s
            mag = None
            
            return accel, gyro, mag
        except:
            return None, None, None

    def parse_vectornav(self, line):
        """Parse VectorNav VNQMR format"""
        try:
            # Typical format: VNQMR,quat,accel,gyro,mag,temp,...
            if not line.startswith('$VNQMR'):
                return None, None, None
            
            parts = line.split(',')
            # This is simplified; adjust based on your actual VectorNav output format
            
            # Parse quaternion
            q = [float(parts[i]) for i in range(1, 5)]
            
            # Parse accel
            accel = [float(parts[i]) for i in range(5, 8)]
            
            # Parse gyro
            gyro = [float(parts[i]) for i in range(8, 11)]
            
            # Parse mag
            mag = [float(parts[i]) for i in range(11, 14)]
            
            return accel, gyro, mag
        except:
            return None, None, None

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
