import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FinalOdomNode(Node):
    """
    Level-3 fusion (Ek Mimari):
      - Prefer LiDAR scan-matching odom (/odometry/icp) when fresh
      - Fall back to EKF odom (/odom)
      - Publish /final_odom
    """

    def __init__(self):
        super().__init__("final_odom_node")
        self.declare_parameter("ekf_odom_topic", "/odom")
        self.declare_parameter("icp_odom_topic", "/odometry/icp")
        self.declare_parameter("out_topic", "/final_odom")
        self.declare_parameter("icp_timeout_s", 0.2)

        self._ekf: Odometry | None = None
        self._icp: Odometry | None = None
        self._ekf_stamp: float = 0.0
        self._icp_stamp: float = 0.0

        self.create_subscription(Odometry, str(self.get_parameter("ekf_odom_topic").value), self._ekf_cb, 10)
        self.create_subscription(Odometry, str(self.get_parameter("icp_odom_topic").value), self._icp_cb, 10)
        self._pub = self.create_publisher(Odometry, str(self.get_parameter("out_topic").value), 10)

        self.create_timer(0.02, self._tick)  # 50 Hz
        self.get_logger().info(
            "final_odom_node ready — "
            f"ekf={str(self.get_parameter('ekf_odom_topic').value)}, "
            f"icp={str(self.get_parameter('icp_odom_topic').value)} -> "
            f"out={str(self.get_parameter('out_topic').value)}"
        )

    def _ekf_cb(self, msg: Odometry) -> None:
        self._ekf = msg
        self._ekf_stamp = time.monotonic()

    def _icp_cb(self, msg: Odometry) -> None:
        self._icp = msg
        self._icp_stamp = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()
        icp_to = float(self.get_parameter("icp_timeout_s").value)
        icp_fresh = self._icp is not None and (now - self._icp_stamp) <= icp_to
        msg = self._icp if icp_fresh else self._ekf
        if msg is None:
            return
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FinalOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

