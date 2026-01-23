#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Int32MultiArray

import serial


class SerialNode(Node):

    def __init__(self):
        super().__init__('serial_node')

        # ---------------- SERIAL ----------------
        self.port = '/dev/ttyUSB0'
        self.baud = 115200

        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.1
            )
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            raise SystemExit

        # ---------------- PUBLISHERS ----------------
        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/wheel_ticks',
            10
        )

        self.imu_pub = self.create_publisher(
            Imu,
            '/imu/raw',
            10
        )

        # ---------------- TIMER ----------------
        self.timer = self.create_timer(0.05, self.read_serial)

    # =====================================================
    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()

            # Example expected:
            # L:123,R:125,AX:0.012,AY:-0.031,GZ:0.0041

            if not line.startswith("L:"):
                return

            parts = line.split(",")

            data = {}
            for p in parts:
                if ":" not in p:
                    continue

                k, v = p.split(":", 1)

                try:
                    data[k] = float(v)
                except ValueError:
                    return


            # ---------------- ENCODERS ----------------
            ticks = Int32MultiArray()
            ticks.data = [
                int(data.get("L", 0)),
                int(data.get("R", 0))
            ]
            self.encoder_pub.publish(ticks)

            # ---------------- IMU ----------------
            imu = Imu()
            imu.header.stamp = self.get_clock().now().to_msg()
            imu.header.frame_id = "imu_link"

            imu.linear_acceleration.x = data.get("AX", 0.0)
            imu.linear_acceleration.y = data.get("AY", 0.0)
            imu.angular_velocity.z = data.get("GZ", 0.0)

            self.imu_pub.publish(imu)

        except Exception as e:
            # NEVER crash node
            self.get_logger().warn(f"Serial parse skipped: {e}")

    # =====================================================


def main():
    rclpy.init()
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
