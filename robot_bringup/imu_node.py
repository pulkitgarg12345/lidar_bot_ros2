import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)

        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)

        self.timer = self.create_timer(0.05, self.read_serial)

        self.get_logger().info("IMU node started")

    def read_serial(self):
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line.startswith("L:"):
            return

        data = {}
        try:
            for part in line.split(","):
                k, v = part.split(":")
                data[k] = float(v)
        except:
            return

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'imu_link'

        # Linear acceleration (m/s^2)
        imu.linear_acceleration.x = data.get("AX", 0.0)
        imu.linear_acceleration.y = data.get("AY", 0.0)
        imu.linear_acceleration.z = 0.0

        # Angular velocity (rad/s)
        imu.angular_velocity.z = data.get("GZ", 0.0)

        # Orientation not provided
        imu.orientation_covariance[0] = -1.0

        self.imu_pub.publish(imu)

    def destroy_node(self):
        self.ser.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
