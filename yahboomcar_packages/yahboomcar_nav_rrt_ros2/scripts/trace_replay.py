#!/usr/bin/env python3

import json

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class TraceReplayNode(Node):
    def __init__(self):
        super().__init__('trace_replay')

        self.declare_parameter('trace_file', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('goal_topic', '/goal_pose')

        self.trace_file = self.get_parameter('trace_file').value
        self.global_frame = self.get_parameter('global_frame').value
        self.navigate_action = self.get_parameter('navigate_action').value

        self.goal_pub = self.create_publisher(PoseStamped, self.get_parameter('goal_topic').value, 10)
        self.nav_client = ActionClient(self, NavigateToPose, self.navigate_action)

        self.goal_index = 0
        self.retry_count = 0
        self.started = False
        self.shutdown_requested = False
        self.goal_points = []

        self.start_timer = self.create_timer(0.5, self.start_replay)

    def load_trace(self):
        if not self.trace_file:
            raise RuntimeError('trace_file parameter is empty')

        with open(self.trace_file, 'r', encoding='utf-8') as trace_handle:
            payload = json.load(trace_handle)

        self.goal_points = [
            goal for goal in payload.get('goals', [])
            if goal.get('status') == 'reached'
        ]
        if not self.goal_points:
            raise RuntimeError('trace file does not contain any reached goals')

    def start_replay(self):
        if self.started:
            return
        self.started = True
        self.start_timer.cancel()

        try:
            self.load_trace()
        except Exception as exc:
            self.get_logger().error(f'Failed to load trace replay file: {exc}')
            self.request_shutdown()
            return

        self.get_logger().info(
            f'Loaded {len(self.goal_points)} reached goals from {self.trace_file}'
        )
        self.nav_client.wait_for_server()
        self.send_next_goal()

    def send_next_goal(self):
        if self.goal_index >= len(self.goal_points):
            self.get_logger().info('Trace replay finished')
            self.request_shutdown()
            return

        goal_entry = self.goal_points[self.goal_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(goal_entry['x'])
        goal_msg.pose.pose.position.y = float(goal_entry['y'])
        goal_msg.pose.pose.orientation.w = 1.0

        pose_msg = PoseStamped()
        pose_msg.header = goal_msg.pose.header
        pose_msg.pose = goal_msg.pose.pose
        self.goal_pub.publish(pose_msg)

        self.get_logger().info(
            f"Replaying goal {self.goal_index + 1}/{len(self.goal_points)} "
            f"id={goal_entry.get('id', -1)} retry={self.retry_count}"
        )

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Replay goal rejected by Nav2')
            self.handle_goal_failure()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.retry_count = 0
            self.goal_index += 1
            self.send_next_goal()
            return

        self.get_logger().warn(
            f'Replay goal failed with nav status {status} at index {self.goal_index}'
        )
        self.handle_goal_failure()

    def handle_goal_failure(self):
        if self.retry_count < 1:
            self.retry_count += 1
            self.get_logger().warn('Retrying current replay goal once')
            self.send_next_goal()
            return

        self.get_logger().warn('Skipping replay goal after one retry')
        self.retry_count = 0
        self.goal_index += 1
        self.send_next_goal()

    def request_shutdown(self):
        if self.shutdown_requested:
            return
        self.shutdown_requested = True
        self.create_timer(0.1, lambda: rclpy.shutdown())


def main(args=None):
    rclpy.init(args=args)
    node = TraceReplayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
