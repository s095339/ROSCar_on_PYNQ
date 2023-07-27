import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan
import math

def  RAD2DEG(x): 
    return math.degrees(x)
class ReadingLaser(Node):

    def __init__(self):
        super().__init__('reading_laser')

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE



        self.subscription= self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos_profile,
        )


    def listener_callback(self,scan):
        count = int(scan.scan_time / scan.time_increment)
        print("count = ",count)
        
        print("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n"%  (scan.header.frame_id, count))
        
        print("[SLLIDAR INFO]: angle_range : [%f, %f]\n" % (RAD2DEG(scan.angle_min),
            RAD2DEG(scan.angle_max)))
        for i in range(count):
            degree = RAD2DEG(scan.angle_min + scan.angle_increment * i)
            print("[SLLIDAR INFO]: angle-distance : [%f, %f]"% (degree, scan.ranges[i]))

        print("--------------------------------------------------------")
        #print(scan)
  


def main(args=None):
    rclpy.init()
    reading_laser = ReadingLaser()                  
    reading_laser.get_logger().info("Hello friend!")
    rclpy.spin(reading_laser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    reading_laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()