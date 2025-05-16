import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,Float32MultiArray
import math
import time

class SLLidarClient(Node):
    def __init__(self):
        super().__init__('obstacle_detection')
        self.subscription1 = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.subscription2 = self.create_subscription(
            Bool,
            'Completed',
            self.start_callback,
            10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, 'lidar_val', 10)
        self.declare_parameter('obstacle_threshold', 0.5)
        self.get_logger().info("SLLIDAR Client Node Started")
        self.val=True
    
    def start_callback(self, msg):
        self.val=True
        
    def scan_callback(self, scan):
        if self.val:
            count = int(scan.scan_time / scan.time_increment)
            obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
            res=[]
            for i in range(count):
                degree = math.degrees(scan.angle_min + scan.angle_increment * i)
                if scan.ranges[i]<0.5:#what is this?
                    res.append(degree)#obs angles and distances 
                    res.append(scan.ranges[i])
                    #self.get_logger().info(f"Obstacle found at angle-distance: [{degree}, {scan.ranges[i]}]")
            msg=Float32MultiArray()
            msg.data=res
            self.get_logger().info(f"{res}")
            self.publisher_.publish(msg)
            self.val=False


def main(args=None):
    rclpy.init(args=args)
    node = SLLidarClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

