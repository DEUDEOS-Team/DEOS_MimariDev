import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
import oto_surus_perspektif



class LaneToWallNode(Node):
    def __init__(self):
        super().__init__('lane_to_wall_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher_ = self.create_publisher(PointCloud2, '/lane_walls', 10)
        self.bridge = CvBridge()
        self.model = oto_surus_perspektif

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except: return

        result = self.model.predict(frame)
        
        

        points_3d = []
        img_h, img_w = frame.shape[:2]

        # Pikselleri Duvara Çevirme
        
        # 1. Modeli çalıştır ve noktaları al (Senin kodundaki 283. satır mantığı)
        lanes_points, _ = self.lane_detector.detect_lanes(frame) 

        points_3d = []
        img_h, img_w = frame.shape[:2]

        # 2. Sadece Sol (1) ve Sağ (2) şerit çizgilerini döngüye al

        # (UltraFastLaneDetector genelde 1 ve 2. indeksleri ego şerit olarak verir)

        for lane_idx in [1, 2]: 
            lane = lanes_points[lane_idx]
            
            # Bu şeritteki her bir nokta için:
            for point in lane:
                u = int(point[0]) # Senin bbox_x dediğin şey aslında bu (Resimdeki X koordinatı)
                v = int(point[1]) # Senin bbox_y dediğin şey aslında bu (Resimdeki Y koordinatı)

                # --- BURADAN SONRASI AYNI (Perspektif Dönüşümü) ---
                
                # Kalibrasyon (Deneme yanılma ile bu sayıları ayarla)
                rel_x = (img_h - v) * 0.015  # İleri mesafe
                rel_y = (img_w/2 - u) * 0.005 # Yana mesafe
                
                if rel_x < 0.2 or rel_x > 5.0: continue

                # Noktayı yükselterek duvar yap
                for z in [0.0, 0.5, 1.0]:
                    points_3d.append([rel_x, rel_y, z])

        # ROS Mesajı Oluştur ve Gönder
        if points_3d:
            self.publish_point_cloud(points_3d)

    def publish_point_cloud(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=3, count=1),
            PointField(name='y', offset=4, datatype=3, count=1),
            PointField(name='z', offset=8, datatype=3, count=1)
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        buffer = []
        for p in points: buffer.append(struct.pack('fff', float(p[0]), float(p[1]), float(p[2])))
        msg.data = b''.join(buffer)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = LaneToWallNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
