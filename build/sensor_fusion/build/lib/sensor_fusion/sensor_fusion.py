import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry  # Assuming encoder data is published as Odometry
from geometry_msgs.msg import Twist  # To publish fused velocity if needed

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscribers
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.encoder_sub = self.create_subscription(
            Odometry, '/wheel_odom', self.encoder_callback, 10)

        # Publisher (optional)
        self.fused_pub = self.create_publisher(Twist, '/fused_velocity', 10)

        self.latest_lidar = None
        self.latest_encoder = None

        self.get_logger().info("Sensor Fusion Node is Running!")

    def lidar_callback(self, msg):
        self.latest_lidar = msg
        self.get_logger().info("Received LiDAR data")
        self.fuse_data()

    def encoder_callback(self, msg):
        self.latest_encoder = msg
        self.get_logger().info("Received Encoder data")
        self.fuse_data()

    def fuse_data(self):
        if self.latest_lidar and self.latest_encoder:
            # Example: Combine data (modify this based on your needs)
            fused_velocity = Twist()
            fused_velocity.linear.x = self.latest_encoder.twist.twist.linear.x  
            fused_velocity.angular.z = 0.0  # Modify based on LiDAR data

            self.fused_pub.publish(fused_velocity)
            self.get_logger().info("Published Fused Data")

def main():
    rclpy.init()
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

