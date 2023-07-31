import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from rclpy.qos import qos_profile_sensor_data

"""
reference : https://answers.ros.org/question/361030/ros2-image-subscriber/

"""
class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('ImageSubscriber')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.ImageReceive_callback,
            qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning

    def ImageReceive_callback(self,img):
        print("In callback")



def main(args=None):
    rclpy.init(args=args)
    Image_Subscriber = ImageSubscriber()
    rclpy.spin(Image_Subscriber)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Image_Subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()