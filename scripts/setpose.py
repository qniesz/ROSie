import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time

rclpy.init()
node = Node("set_initial_pose")
pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
time.sleep(1)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = "map"
msg.header.stamp = node.get_clock().now().to_msg()
msg.pose.pose.position.x = 0.0
msg.pose.pose.position.y = 0.0
msg.pose.pose.orientation.w = 1.0
msg.pose.covariance[0] = 0.25
msg.pose.covariance[7] = 0.25
msg.pose.covariance[35] = 0.068
pub.publish(msg)
time.sleep(1)
node.destroy_node()
rclpy.shutdown()
print("Initial pose set")
