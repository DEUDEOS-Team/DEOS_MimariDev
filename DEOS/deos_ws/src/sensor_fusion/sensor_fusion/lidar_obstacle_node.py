import json
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__("lidar_obstacle_node")

        self.declare_parameter("cluster_epsilon_m", 0.5)
        self.declare_parameter("cluster_min_points", 5)
        self.declare_parameter("max_distance_m", 20.0)
        self.declare_parameter("corridor_half_width_m", 3.0)

        self._eps = float(self.get_parameter("cluster_epsilon_m").value)
        self._min_pts = int(self.get_parameter("cluster_min_points").value)
        self._max_dist = float(self.get_parameter("max_distance_m").value)
        self._corridor_hw = float(self.get_parameter("corridor_half_width_m").value)

        self.create_subscription(PointCloud2, "/points_downsampled", self._cloud_cb, 10)
        self._pub = self.create_publisher(String, "/perception/lidar_obstacles", 10)
        self.get_logger().info("lidar_obstacle_node ready")

    def _cloud_cb(self, msg: PointCloud2) -> None:
        pts = self._unpack_xyz(msg)
        if pts is None or len(pts) == 0:
            self._pub.publish(String(data="[]"))
            return

        mask = (pts[:, 0] > 0.3) & (pts[:, 0] < self._max_dist) & (np.abs(pts[:, 1]) < self._corridor_hw)
        pts = pts[mask]
        if len(pts) == 0:
            self._pub.publish(String(data="[]"))
            return

        clusters = self._euclidean_cluster(pts[:, :2])
        obstacles = []
        for cl in clusters:
            dist = float(np.min(cl[:, 0]))
            lat = float(np.mean(cl[:, 1]))
            diameter = float(max(np.max(cl[:, 0]) - np.min(cl[:, 0]), np.max(cl[:, 1]) - np.min(cl[:, 1])))
            obstacles.append(
                {
                    "kind": self._classify(diameter),
                    "confidence": 0.7,
                    "distance_m": dist,
                    "lateral_m": lat,
                    "bbox_px": None,
                }
            )

        self._pub.publish(String(data=json.dumps(obstacles)))

    @staticmethod
    def _classify(diameter: float) -> str:
        if diameter < 0.4:
            return "cone"
        if diameter < 1.5:
            return "unknown"
        return "barrier"

    def _euclidean_cluster(self, pts_xy: np.ndarray) -> list[np.ndarray]:
        visited = np.zeros(len(pts_xy), dtype=bool)
        clusters: list[np.ndarray] = []

        for i in range(len(pts_xy)):
            if visited[i]:
                continue

            dists = np.linalg.norm(pts_xy - pts_xy[i], axis=1)
            nbrs = np.where(dists < self._eps)[0]
            if len(nbrs) < self._min_pts:
                visited[i] = True
                continue

            cluster_idx: set[int] = set(nbrs.tolist())
            queue = list(nbrs)
            while queue:
                j = queue.pop()
                if visited[j]:
                    continue
                visited[j] = True
                d2 = np.linalg.norm(pts_xy - pts_xy[j], axis=1)
                new_nbrs = np.where(d2 < self._eps)[0]
                if len(new_nbrs) >= self._min_pts:
                    for nb in new_nbrs:
                        if nb not in cluster_idx:
                            cluster_idx.add(int(nb))
                            queue.append(int(nb))

            clusters.append(pts_xy[list(cluster_idx)])

        return clusters

    def _unpack_xyz(self, msg: PointCloud2) -> np.ndarray | None:
        try:
            fmap = {f.name: f.offset for f in msg.fields}
            if not all(k in fmap for k in ("x", "y", "z")):
                return None

            ox, oy, oz = fmap["x"], fmap["y"], fmap["z"]
            step = msg.point_step
            n = msg.width * msg.height
            data = bytes(msg.data)
            pts = np.empty((n, 3), dtype=np.float32)
            for i in range(n):
                base = i * step
                pts[i, 0] = struct.unpack_from("f", data, base + ox)[0]
                pts[i, 1] = struct.unpack_from("f", data, base + oy)[0]
                pts[i, 2] = struct.unpack_from("f", data, base + oz)[0]
            return pts[np.isfinite(pts).all(axis=1)]
        except Exception as e:
            self.get_logger().error(f"PointCloud2 unpack: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

