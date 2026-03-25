#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, PointStamped, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus


class MultipointNavigation(Node):
    def __init__(self):
        super().__init__('multipoint_navigation')
        self.InitialParam()
        self.pub_mark = self.create_publisher(MarkerArray, 'path_point', 100)
        self.sub_click = self.create_subscription(PointStamped, 'clicked_point', self.press_callback, 10)
        self.pub_goal = self.create_publisher(PoseStamped, 'goal_pose', 1)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sub_initialpose = self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.initialpose_callback, 10)
        self.pub_rtabinitPose = self.create_publisher(PoseWithCovarianceStamped, 'rtabmap/initialpose', 10)
        self.pub_cmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_JoyState = self.create_subscription(Bool, '/JoyState', self.JoyStateCallback, 10)
        self.goal_handle = None

    def JoyStateCallback(self, msg):
        if msg.data:
            self.cancel_marker()
            self.pub_cmdVel.publish(Twist())

    def InitialParam(self):
        self.markerArray = MarkerArray()
        self.count = 0
        self.index = 0
        self.try_again = 1

    def initialpose_callback(self, msg):
        self.cancel_marker()
        self.pub_rtabinitPose.publish(msg)

    def cancel_marker(self):
        self.markerArray = MarkerArray()
        marker = Marker()
        marker.action = marker.DELETEALL
        self.markerArray.markers.append(marker)
        self.pub_mark.publish(self.markerArray)
        self.InitialParam()
        if self.goal_handle:
            self.goal_handle.cancel_goal_async()

    def press_callback(self, msg):
        self.get_logger().info(f'Add a new target point {self.count}.')
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        marker.pose.position.z = msg.point.z
        marker.text = str(self.count)
        self.markerArray.markers.append(marker)
        for i, m in enumerate(self.markerArray.markers):
            m.id = i
        self.pub_mark.publish(self.markerArray)
        if self.count == 0:
            self.PubTargetPoint(msg.point.x, msg.point.y)
            self.index += 1
        self.count += 1

    def PubTargetPoint(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        self.nav_client.wait_for_server()
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        if self.count == 0:
            return
        result = future.result()
        status = result.status
        self.get_logger().info('Get the status of reaching the target point!')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.try_again = 1
            if self.index == self.count:
                self.get_logger().info(f'Reach the target point {self.index - 1}.')
                self.index = 0
                x = self.markerArray.markers[self.index].pose.position.x
                y = self.markerArray.markers[self.index].pose.position.y
                self.PubTargetPoint(x, y)
                self.index += 1
            elif self.index < self.count:
                self.get_logger().info(f'Reach the target point {self.index - 1}.')
                x = self.markerArray.markers[self.index].pose.position.x
                y = self.markerArray.markers[self.index].pose.position.y
                self.PubTargetPoint(x, y)
                self.index += 1
        else:
            self.get_logger().warn(f'Can not reach the target point {self.index - 1}.')
            if self.try_again == 1:
                self.get_logger().warn(f'trying reach the target point {self.index - 1} again!')
                x = self.markerArray.markers[self.index - 1].pose.position.x
                y = self.markerArray.markers[self.index - 1].pose.position.y
                self.PubTargetPoint(x, y)
                self.try_again = 0
            elif self.index < len(self.markerArray.markers):
                self.get_logger().warn(f'try reach the target point {self.index - 1} failed! reach next point.')
                if self.index == self.count:
                    self.index = 0
                x = self.markerArray.markers[self.index].pose.position.x
                y = self.markerArray.markers[self.index].pose.position.y
                self.PubTargetPoint(x, y)
                self.index += 1
                self.try_again = 1


def main(args=None):
    rclpy.init(args=args)
    node = MultipointNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
