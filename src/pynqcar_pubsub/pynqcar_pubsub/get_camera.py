import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
"""
reference : 
https://answers.ros.org/question/361030/ros2-image-subscriber/
https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
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
        self.br = CvBridge()

    def ImageReceive_callback(self,img):
        current_frame = self.br.imgmsg_to_cv2(img,desired_encoding='passthrough')
        """
        The shape of current frame is [480,640,2]
        What we need is a grayscale image with shape 480,640,1
        I just convert the image from 2 channel to 1 channel with np.mean(),
        probably not a best way
        """
        
        frame_gray = np.mean(current_frame, axis=2).astype(np.uint8)
        #frame_gray = current_frame[:,:,0].astype(np.uint8)
        #frame_ch2 = current_frame[:,:,1].astype(np.uint8)
        #print(current_frame)
        #print(frame_gray.shape)

        #frame_ch1, frame_ch2 = cv2.split(current_frame)
        cv2.imshow("camera_merged", frame_gray)
        #cv2.imshow("camera_merged", frame_ch1)
        #cv2.imshow("camera_merged", frame_ch1)
        #cv2.imshow("camera_ch1", frame_ch1)
        #cv2.imshow("camera_ch2", frame_ch2)
        cv2.waitKey(1)


def main(args=None):
    print("open image sub")
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