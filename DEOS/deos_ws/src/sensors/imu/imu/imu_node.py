import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

from deos_algorithms.ros_topic_layout import build_deos_topics

from deos_logging.logger import DeosLogger


class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.logger = DeosLogger(self.get_logger(), "imu_node")
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_type', 'mpu9250')  # 'mpu9250', 'vectornav', 'xsens'
        self.declare_parameter('deos_root', '/deos')
        _T = build_deos_topics(str(self.get_parameter('deos_root').value))
        self.declare_parameter('imu_topic', _T['sensors_imu'])

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.imu_type = self.get_parameter('imu_type').value

        # Basit yaw integrasyonu: orientation_covariance=-1 ise EKF orientation almaz,
        # ama perception_fusion_node quaternion'dan heading hesaplar.
        # Gyro Z integrasyon ile yaklaşık heading üret (drift birikir ama kısa görevlerde kabul edilebilir).
        self._yaw_rad: float = 0.0
        self._last_imu_t: float = time.monotonic()

        # Serial setup
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.logger.info(f"IMU connected on {port} @ {baudrate} baud (type: {self.imu_type})")
        except Exception as e:
            self.logger.error(f"Failed to open IMU port: {e}")
            self.ser = None
            return
        
        # Publisher
        self.publisher_ = self.create_publisher(Imu, str(self.get_parameter('imu_topic').value), 10)
        
        # Timer for read loop
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100 Hz
        self.logger.info("IMU node started")

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

            now = time.monotonic()
            dt = min(now - self._last_imu_t, 0.05)  # max 50ms to avoid large jumps
            self._last_imu_t = now

            # Yaw integrasyonu: gyro[2] = z-ekseni açısal hız (rad/s)
            self._yaw_rad += float(gyro[2]) * dt

            # Quaternion (yalnızca yaw; pitch/roll sıfır varsayılır — düz zemin)
            half = self._yaw_rad / 2.0
            qw = math.cos(half)
            qx = 0.0
            qy = 0.0
            qz = math.sin(half)

            # Create Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.frame_id

            # Orientation (gyro integrasyon — EKF bunu iç model ile birleştirir)
            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz

            # Accelerometer (m/s²)
            imu_msg.linear_acceleration.x = accel[0]
            imu_msg.linear_acceleration.y = accel[1]
            imu_msg.linear_acceleration.z = accel[2]

            # Gyroscope (rad/s)
            imu_msg.angular_velocity.x = gyro[0]
            imu_msg.angular_velocity.y = gyro[1]
            imu_msg.angular_velocity.z = gyro[2]

            # Covariance
            imu_msg.linear_acceleration_covariance = [0.01, 0, 0,  0, 0.01, 0,  0, 0, 0.01]
            imu_msg.angular_velocity_covariance     = [0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.005]
            # Orientation covariance: gyro integrasyonu uzun vadede drift biriktirir
            imu_msg.orientation_covariance          = [0.1, 0, 0,  0, 0.1, 0,  0, 0, 0.05]
            
            self.publisher_.publish(imu_msg)
        
        except Exception as e:
            self.logger.error(f"IMU parsing error: {e}")

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
