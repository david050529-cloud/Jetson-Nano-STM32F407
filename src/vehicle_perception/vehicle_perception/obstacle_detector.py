import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sklearn.cluster import DBSCAN

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.sub = self.create_subscription(PointCloud2, '/lidar/points', self.cloud_callback, 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/perception/obstacles_markers', 10)

    def cloud_callback(self, msg):
        # 转换点云到numpy数组 (x, y, z)
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        if len(points) == 0:
            return
        # 简单地面滤波（假设地面在z<0.2米）
        non_ground = points[points[:, 2] > 0.2]
        if len(non_ground) == 0:
            return
        # 聚类
        clustering = DBSCAN(eps=0.3, min_samples=5).fit(non_ground[:, :2])
        labels = clustering.labels_
        marker_array = MarkerArray()
        for label in set(labels):
            if label == -1:
                continue
            cluster_points = non_ground[labels == label]
            center = np.mean(cluster_points[:, :2], axis=0)
            marker = Marker()
            marker.header = msg.header
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = 0.5
            marker.scale.x = marker.scale.y = marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker_array.markers.append(marker)
        self.pub_markers.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)