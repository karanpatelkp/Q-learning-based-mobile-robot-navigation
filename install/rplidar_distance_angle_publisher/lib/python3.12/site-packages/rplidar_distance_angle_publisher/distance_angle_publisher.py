import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import time
n=0
class DistanceAnglePublisher(Node):
    def __init__(self):
        super().__init__('distance_angle_publisher')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )
        self.publisher_ = self.create_publisher(Float32MultiArray, '/rplidar_distance_angle', 10)
        self.get_logger().info('Distance and Angle Publisher Node Initialized')

    def laser_scan_callback(self, msg):
        global n
        data = Float32MultiArray()
        
        # Extend the data with the ranges from the LaserScan message
        '''data.data = msg.ranges
        
        # Publish the distances
        self.publisher_.publish([data,n])
        n+=1
        self.get_logger().info(f'Published distances: {data.data}')'''
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            data.data.extend([distance, msg.angle_increment,msg.angle_min,msg.angle_max,n])  # Add distance and angle pairs
            n+=1
        self.publisher_.publish(data)
        n=0
        time.sleep(2)

        
def main(args=None):
    rclpy.init(args=args)
    node = DistanceAnglePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

