#!/usr/bin/env python3

# ROS2 conversion of filter.py
from copy import copy
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from numpy import array, vstack, delete
from functions_ros2 import gridValue, informationGain
from sklearn.cluster import MeanShift
from yahboomcar_nav_rrt.msg import PointArray


class FilterNode(Node):
    def __init__(self):
        super().__init__('filter')

        # Declare parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('info_radius', 1.0)
        self.declare_parameter('costmap_clearing_threshold', 70.0)
        self.declare_parameter('goals_topic', '/detected_points')
        self.declare_parameter('n_robots', 1)
        self.declare_parameter('namespace', '')
        self.declare_parameter('namespace_init_count', 1)
        self.declare_parameter('rate', 100.0)

        # Get parameters
        map_topic = self.get_parameter('map_topic').value
        self.info_radius = self.get_parameter('info_radius').value
        self.costmap_clearing_threshold = self.get_parameter('costmap_clearing_threshold').value
        goals_topic = self.get_parameter('goals_topic').value
        self.n_robots = self.get_parameter('n_robots').value
        namespace = self.get_parameter('namespace').value
        namespace_init_count = self.get_parameter('namespace_init_count').value

        # Initialize variables
        self.frontiers = []
        self.mapData = OccupancyGrid()
        self.globalmaps = [OccupancyGrid() for _ in range(self.n_robots)]

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.mapCallBack, 10)

        for i in range(self.n_robots):
            if namespace:
                topic = '/' + namespace + str(i + namespace_init_count) + '/move_base/global_costmap/costmap'
            else:
                topic = '/move_base/global_costmap/costmap'
            self.create_subscription(OccupancyGrid, topic, self.globalMapCallBack, 10)

        self.goals_sub = self.create_subscription(
            PointStamped, goals_topic, self.goalsCallBack, 10)

        # Publishers
        self.frontiers_pub = self.create_publisher(Marker, 'frontiers', 10)
        self.centroids_pub = self.create_publisher(Marker, 'centroids', 10)
        self.filtered_pub = self.create_publisher(PointArray, 'filtered_points', 10)

        self.get_logger().info('Filter node initialized')

    def goalsCallBack(self, data):
        try:
            transformed = self.tf_buffer.transform(data, self.mapData.header.frame_id)
            x = array([transformed.point.x, transformed.point.y])
            if len(self.frontiers) > 0:
                self.frontiers = vstack((self.frontiers, [x]))
            else:
                self.frontiers = [x]
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {e}')

    def mapCallBack(self, data):
        self.mapData = data

    def globalMapCallBack(self, data):
        if self.n_robots == 1:
            self.globalmaps[0] = data
        else:
            # Extract robot index from topic
            topic = data.header.frame_id
            for i in range(self.n_robots):
                self.globalmaps[i] = data
                break

    def run(self):
        # Wait for map
        while len(self.mapData.data) < 1 and rclpy.ok():
            self.get_logger().info('Waiting for map')
            rclpy.spin_once(self, timeout_sec=0.1)

        # Wait for global costmaps
        for i in range(self.n_robots):
            while len(self.globalmaps[i].data) < 1 and rclpy.ok():
                self.get_logger().info('Waiting for global costmap')
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Map and global costmaps received')

        # Wait for frontiers
        while len(self.frontiers) < 1 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        # Initialize markers
        points = Marker()
        points.header.frame_id = self.mapData.header.frame_id
        points.ns = "markers2"
        points.id = 0
        points.type = Marker.POINTS
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = 0.2
        points.scale.y = 0.2
        points.color.r = 1.0
        points.color.g = 1.0
        points.color.b = 0.0
        points.color.a = 1.0

        points_clust = Marker()
        points_clust.header.frame_id = self.mapData.header.frame_id
        points_clust.ns = "markers3"
        points_clust.id = 4
        points_clust.type = Marker.POINTS
        points_clust.action = Marker.ADD
        points_clust.pose.orientation.w = 1.0
        points_clust.scale.x = 0.2
        points_clust.scale.y = 0.2
        points_clust.color.r = 0.0
        points_clust.color.g = 1.0
        points_clust.color.b = 0.0
        points_clust.color.a = 1.0

        rate = self.create_rate(100)

        # Main loop
        while rclpy.ok():
            centroids = []
            front = copy(self.frontiers)

            # Clustering
            if len(front) > 1:
                ms = MeanShift(bandwidth=0.3)
                ms.fit(front)
                centroids = ms.cluster_centers_
            elif len(front) == 1:
                centroids = front

            self.frontiers = copy(centroids)

            # Clear old frontiers
            z = 0
            while z < len(centroids):
                cond = False
                temppoint = PointStamped()
                temppoint.header.frame_id = self.mapData.header.frame_id
                temppoint.header.stamp = rclpy.time.Time().to_msg()
                temppoint.point.x = centroids[z][0]
                temppoint.point.y = centroids[z][1]
                temppoint.point.z = 0.0

                for i in range(self.n_robots):
                    try:
                        transformed = self.tf_buffer.transform(
                            temppoint, self.globalmaps[i].header.frame_id)
                        x = array([transformed.point.x, transformed.point.y])
                        cond = (gridValue(self.globalmaps[i], x) > self.costmap_clearing_threshold) or cond
                    except:
                        pass

                if cond or informationGain(self.mapData, [centroids[z][0], centroids[z][1]], self.info_radius*0.5) < 0.2:
                    centroids = delete(centroids, z, axis=0)
                    z -= 1
                z += 1

            # Publish filtered points
            arraypoints = PointArray()
            for i in centroids:
                tempPoint = Point()
                tempPoint.x = float(i[0])
                tempPoint.y = float(i[1])
                tempPoint.z = 0.0
                arraypoints.points.append(tempPoint)
            self.filtered_pub.publish(arraypoints)

            # Publish markers
            points.header.stamp = self.get_clock().now().to_msg()
            points.points = []
            for q in range(len(self.frontiers)):
                p = Point()
                p.x = float(self.frontiers[q][0])
                p.y = float(self.frontiers[q][1])
                p.z = 0.0
                points.points.append(p)
            self.frontiers_pub.publish(points)

            points_clust.header.stamp = self.get_clock().now().to_msg()
            points_clust.points = []
            for q in range(len(centroids)):
                p = Point()
                p.x = float(centroids[q][0])
                p.y = float(centroids[q][1])
                p.z = 0.0
                points_clust.points.append(p)
            self.centroids_pub.publish(points_clust)

            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
