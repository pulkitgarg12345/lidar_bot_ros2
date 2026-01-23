#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster


# ================= PARAMETERS =================
WHEEL_RADIUS = 0.0325
WHEEL_BASE   = 0.11
TICKS_PER_REV = 290

RAD_PER_TICK = (2.0 * math.pi) / TICKS_PER_REV


class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_node')

        # ---------------- Publishers ----------------
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- Subscriber ----------------
        self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.tick_callback,
            10
        )

        # ---------------- Robot State ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_ticks = 0
        self.right_ticks = 0
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0

        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        # ---------------- Timer ----------------
        self.timer = self.create_timer(0.05, self.publish_all)

        self.get_logger().info("Odom node running (serial-free)")

    # =================================================
    def tick_callback(self, msg):
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]

    # =================================================
    def publish_all(self):

        now = self.get_clock().now().to_msg()

        # tick difference
        dL = self.left_ticks - self.prev_left_ticks
        dR = self.right_ticks - self.prev_right_ticks

        self.prev_left_ticks = self.left_ticks
        self.prev_right_ticks = self.right_ticks

        # wheel position
        self.left_wheel_pos += dL * RAD_PER_TICK
        self.right_wheel_pos += dR * RAD_PER_TICK

        # ---------------- differential drive ----------------
        d_left  = dL * RAD_PER_TICK * WHEEL_RADIUS
        d_right = dR * RAD_PER_TICK * WHEEL_RADIUS

        d_center = (d_left + d_right) / 2.0
        d_theta  = (d_right - d_left) / WHEEL_BASE

        self.theta += d_theta
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        # ---------------- joint states ----------------
        joint = JointState()
        joint.header.stamp = now
        joint.name = ['left_wheel_joint', 'right_wheel_joint']
        joint.position = [self.left_wheel_pos, self.right_wheel_pos]
        self.joint_pub.publish(joint)

        # ---------------- quaternion ----------------
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # ---------------- TF ----------------
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        # ---------------- odometry ----------------
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = OdomNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
