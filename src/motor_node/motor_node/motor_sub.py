import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class MotorSubscriber(Node):
    def __init__(self):
        super().__init__('motor_sub')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'mot_val',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Int32MultiArray, 'mot_sign', 10)
        try:
            self.arduino = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port: {e}")

    def listener_callback(self, msg):
        val=msg.data
        if len(msg.data) != 4:
            self.get_logger().error(f"Received incomplete data: {msg.data}")
            return
        command = f"{val[0]},{val[1]},{val[2]},{val[3]}\n"
        self.arduino.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent: {command.strip()}")
        msg1=Int32MultiArray()
        msg1.data=[int(val[i]/abs(val[i])) if val[i] else 1 for i in range(4)]
        self.publisher_.publish(msg1)

def main(args=None):
    rclpy.init(args=args)
    motor_subscriber = MotorSubscriber()
    rclpy.spin(motor_subscriber)
    motor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
