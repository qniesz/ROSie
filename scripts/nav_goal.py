import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import sys

class NavGoalClient(Node):
    def __init__(self):
        super().__init__('nav_goal_client')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y):
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available!')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self.get_logger().info(f'Sending goal: x={x}, y={y}')
        future = self._client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        self.get_logger().info('Goal accepted! Waiting for result...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        result = result_future.result()
        if result:
            self.get_logger().info(f'Navigation result: status={result.status}')
        else:
            self.get_logger().info('Navigation timed out (120s)')

    def feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {dist:.2f}m')

rclpy.init()
x = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
client = NavGoalClient()
client.send_goal(x, y)
client.destroy_node()
rclpy.shutdown()
