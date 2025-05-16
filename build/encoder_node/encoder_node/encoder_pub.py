import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from gpiozero import Button
import time
import math

class Encoder_publish(Node):
    def __init__(self):
        super().__init__('encoder_pub')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'enc_val', 10)
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'mot_sign',
            self.listener_callback,
            10)
        self.sign=[1,1,1,1]
        self.timer_period = 0.03
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.count=[0,0,0,0]
        self.encoder=[Button(16), Button(23), Button(24), Button(25)]
        self.diameter = 0.065  # meters
        self.ppr = 20          # Pulses per revolution
        self.prev = [0,0,0,0]
        #self.N = [0,0,0,0]
        for i in range(4):
            self.encoder[i].when_pressed=lambda i=i: self.enco_call(i)
    
    def listener_callback(self,msg):
        val=msg.data
        if len(msg.data) != 4:
            self.get_logger().error(f"Received incomplete data: {msg.data}")
            return
        for i in range(4):
            if val[i]<0:
                self.sign[i]=-1
            else:
                self.sign[i]=1
        
    def timer_callback(self):
        #distance=[round(self.count[i]*math.pi*3.25*0.1,3) for i in range(4)] #cm
        #print(f"Counts: {count} Distance: {distance} cm")
        velocity = [(self.sign[i] * math.pi * self.diameter * self.count[i]) / (self.ppr * self.timer_period) for i in range(4)]
        self.count=[0,0,0,0]
        for i in range(4):
            if time.time()-self.prev[i]>1:
                velocity[i]=0
        msg = Float32MultiArray()
        msg.data = velocity
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1
        
    def enco_call(self,no):
        self.count[no] += 1  # Increment pulse count
        '''if self.prev[no]:  # Ensure prev time exists
            self.N[no] = time.time() - self.prev[no]
            if self.N[no] != 0:
                self.N[no] = 1 / self.N[no]  # Convert to frequency'''
        self.prev[no] = time.time()

def main(args=None):
    rclpy.init(args=args)
    encoder_publish = Encoder_publish()
    rclpy.spin(encoder_publish)
    encoder_publish.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
