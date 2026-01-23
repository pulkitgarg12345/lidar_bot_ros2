import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import time


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # ---------------- ROBOT PARAMS ----------------
        self.WHEEL_DIAMETER = 0.065   # meters
        self.WHEEL_BASE = 0.11        # meters
        self.TICKS_PER_REV = 290

        self.DIST_PER_TICK = math.pi * self.WHEEL_DIAMETER / self.TICKS_PER_REV

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left = None
        self.prev_right = None
        self.prev_time = time.time()

        # ---------------- ROS ----------------
        self.sub = self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.encoder_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("Odometry node started")

    def encoder_callback(self, msg):
        left = msg.data[0]
        right = msg.data[1]

        if self.prev_left is None:
            self.prev_left = left
            self.prev_right = right
            return

        # ---------------- TIME ----------------
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now

        # ---------------- DELTA TICKS ----------------
        dL = left - self.prev_left
        dR = right - self.prev_right
        self.prev_left = left
        self.prev_right = right

        # ---------------- DISTANCE ----------------
        dist_L = dL * self.DIST_PER_TICK
        dist_R = dR * self.DIST_PER_TICK

        dist = (dist_L + dist_R) / 2.0
        dtheta = (dist_R - dist_L) / self.WHEEL_BASE

        # ---------------- POSE UPDATE ----------------
        self.theta += dtheta
        self.x += dist * math.cos(self.theta)
        self.y += dist * math.sin(self.theta)

        # ---------------- ODOM MSG ----------------
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        q = self.yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = dist / dt if dt > 0 else 0.0
        odom.twist.twist.angular.z = dtheta / dt if dt > 0 else 0.0

        self.odom_pub.publish(odom)
        self.publish_tf(odom)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def publish_tf(self, odom):
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
