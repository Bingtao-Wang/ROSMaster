#!/usr/bin/env python3

# ROS2 conversion of assigner.py
from copy import copy
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from yahboomcar_nav_rrt.msg import PointArray
from time import time
from numpy import array, inf, all as All
from numpy import linalg as LA
from functions_ros2 import robot, informationGain, discount
from numpy.linalg import norm


class AssignerNode(Node):
    def __init__(self):
        super().__init__('assigner')

        # Declare parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('info_radius', 1.0)
        self.declare_parameter('info_multiplier', 3.0)
        self.declare_parameter('hysteresis_radius', 3.0)
        self.declare_parameter('hysteresis_gain', 2.0)
        self.declare_parameter('frontiers_topic', '/filtered_points')
        self.declare_parameter('n_robots', 1)
        self.declare_parameter('namespace', '')
        self.declare_parameter('namespace_init_count', 1)

        # Get parameters
        map_topic = self.get_parameter('map_topic').value
        self.info_radius = self.get_parameter('info_radius').value
        self.info_multiplier = self.get_parameter('info_multiplier').value
        self.hysteresis_radius = self.get_parameter('hysteresis_radius').value
        self.hysteresis_gain = self.get_parameter('hysteresis_gain').value
        frontiers_topic = self.get_parameter('frontiers_topic').value
        n_robots = self.get_parameter('n_robots').value
        namespace = self.get_parameter('namespace').value
        namespace_init_count = self.get_parameter('namespace_init_count').value

        # Initialize variables
        self.frontiers = []
        self.mapData = OccupancyGrid()
        self.globalmaps = []

        # Create subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, map_topic, self.mapCallBack, 10)
        self.frontiers_sub = self.create_subscription(
            PointArray, frontiers_topic, self.callBack, 10)

        # Initialize robots
        self.robots = []
        for i in range(n_robots):
            if namespace:
                robot_name = namespace + str(i + namespace_init_count)
            else:
                robot_name = '' if n_robots == 1 else str(i + namespace_init_count)
            self.robots.append(robot(self, robot_name))

        self.get_logger().info('Assigner node initialized')

    def callBack(self, data):
        self.frontiers = []
        for point in data.points:
            self.frontiers.append(array([point.x, point.y]))

    def mapCallBack(self, data):
        self.mapData = data

    def run(self):
        # Wait for frontiers and map
        while len(self.frontiers) < 1 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        while len(self.mapData.data) < 1 and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

        # Send initial goals
        for robot in self.robots:
            robot.sendGoal(robot.getPosition())

        rate = self.create_rate(100)

        # Main loop
        while rclpy.ok():
            centroids = copy(self.frontiers)

            # Get information gain
            infoGain = []
            for ip in range(len(centroids)):
                infoGain.append(informationGain(
                    self.mapData, [centroids[ip][0], centroids[ip][1]], self.info_radius))

            # Get available/busy robots
            na = []  # available
            nb = []  # busy
            for i in range(len(self.robots)):
                if self.robots[i].getState() == 1:
                    nb.append(i)
                else:
                    na.append(i)

            self.get_logger().info(f"Available robots: {na}")

            # Discount information gain
            for i in nb + na:
                infoGain = discount(self.mapData, self.robots[i].assigned_point,
                                    centroids, infoGain, self.info_radius)

            # Calculate revenue for each robot-frontier pair
            revenue_record = []
            centroid_record = []
            id_record = []

            for ir in na:
                for ip in range(len(centroids)):
                    cost = norm(self.robots[ir].getPosition() - centroids[ip])
                    information_gain = infoGain[ip]
                    if norm(self.robots[ir].getPosition() - centroids[ip]) <= self.hysteresis_radius:
                        information_gain *= self.hysteresis_gain
                    revenue = information_gain * self.info_multiplier - cost
                    revenue_record.append(revenue)
                    centroid_record.append(centroids[ip])
                    id_record.append(ir)

            # If no available robots, check busy ones
            if len(na) < 1:
                revenue_record = []
                centroid_record = []
                id_record = []
                for ir in nb:
                    for ip in range(len(centroids)):
                        cost = norm(self.robots[ir].getPosition() - centroids[ip])
                        information_gain = infoGain[ip]
                        if norm(self.robots[ir].getPosition() - centroids[ip]) <= self.hysteresis_radius:
                            information_gain *= self.hysteresis_gain
                        if norm(centroids[ip] - self.robots[ir].assigned_point) < self.hysteresis_radius:
                            information_gain = informationGain(
                                self.mapData, [centroids[ip][0], centroids[ip][1]], self.info_radius) * self.hysteresis_gain
                        revenue = information_gain * self.info_multiplier - cost
                        revenue_record.append(revenue)
                        centroid_record.append(centroids[ip])
                        id_record.append(ir)

            self.get_logger().info(f"Revenue: {revenue_record}")
            self.get_logger().info(f"Centroids: {centroid_record}")
            self.get_logger().info(f"Robot IDs: {id_record}")

            # Assign goal to winner
            if len(id_record) > 0:
                winner_id = revenue_record.index(max(revenue_record))
                self.robots[id_record[winner_id]].sendGoal(centroid_record[winner_id])
                self.get_logger().info(f"Robot {id_record[winner_id]} assigned to {centroid_record[winner_id]}")

            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = AssignerNode()
    node.run()
    rclpy.shutdown()


if __name__ == '__main__':
    main()







