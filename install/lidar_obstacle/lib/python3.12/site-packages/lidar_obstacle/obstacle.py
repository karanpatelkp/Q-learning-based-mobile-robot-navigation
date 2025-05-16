import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import serial
import time
arduino=serial.Serial('/dev/ttyACM0',9600,timeout=1)

class SLLidarClient(Node):
    def __init__(self):
        super().__init__('obstacle_detection')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.declare_parameter('obstacle_threshold', 0.5)
        self.get_logger().info("SLLIDAR Client Node Started")
        arduino.write(b'1')

    def scan_callback(self, scan):
        count = int(scan.scan_time / scan.time_increment)
        self.get_logger().info(f"Received laser scan {scan.header.frame_id} [{count}] readings.")
        self.get_logger().info(f"Angle range: [{math.degrees(scan.angle_min)}, {math.degrees(scan.angle_max)}]")
        
        obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        st = [0] * 21
        st1 = [0] * 21
        c = 1
        
        for i in range(count):
            degree = math.degrees(scan.angle_min + scan.angle_increment * i)
            if(degree>-3 and degree<3 and scan.ranges[i]<0.5):
                arduino.write(b'0') #Stops the motors
            '''if scan.ranges[i] < obstacle_threshold:
                if c == 21:
                    self.get_logger().info(f"Obstacle found at angle-distance: [{st[20]}, {st1[20]}]")
                    c = 1
                st[c] = (st[c-1] * (c-1) + degree) / c
                st1[c] = (st1[c-1] * (c-1) + scan.ranges[i]) / c
                c += 1 '''
        
        time.sleep(1)  # Simulating the sleep_for(1s) in C++


def main(args=None):
    rclpy.init(args=args)
    node = SLLidarClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

