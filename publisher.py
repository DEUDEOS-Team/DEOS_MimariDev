# ~/publisher.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('rpi_motor_publisher')

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',   # standart topic ismi
            10
        )

        self.timer = self.create_timer(0.1, self.callback)  # 10 Hz

    def callback(self):
        msg = Twist()

        # Buraya kendi değerlerinizi girin
        msg.linear.x  = 1.0    # motor hızı  → ileri: +, geri: -
        msg.angular.z = 0.0    # direksiyon  → sol: +, sağ: -

        self.pub.publish(msg)
        self.get_logger().info(
            f'Hız: {msg.linear.x:.2f} | Direksiyon: {msg.angular.z:.2f}'
        )

def main():
    rclpy.init()
    rclpy.spin(MotorPublisher())

main()