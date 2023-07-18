import rclpy
from rclpy.node import Node
import threading

from std_msgs.msg import String


class SpeedSubscriber(Node):

    def __init__(self):
        super().__init__('SpeedSubscriber')
        self.subscription = self.create_subscription(
            String,
            'speed_cmd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Current speed: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    
    minimal_subscriber = SpeedSubscriber()
    rclpy.spin(minimal_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()