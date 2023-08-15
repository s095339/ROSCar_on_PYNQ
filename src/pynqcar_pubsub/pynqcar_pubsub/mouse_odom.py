import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion
import struct

class MouseOdom(Node):

    def __init__(self):
        super().__init__('mouse_odom')
        self.publisher_ = self.create_publisher(Odometry, 'mouse_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        #timer_period = 0.1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)

        # init variables
        # Car spec, assume car goes in x direction
        self.lx = 0.075 # meters
        self.ly = 0.0975 # meters
        self.r = 0.0275 # meters
        self.x = 0.0  # 
        self.y = 0.0  # current orientation
        self.th = 0.0 # 
        self.file = open( "/dev/input/mice", "rb" ) # open mouse
        self.odom = Odometry()
        self.t = TransformStamped()

        # init done message
        self.get_logger().info('Publish /mouse_odom(Odometry)')

        while 1:
            self.getMouseEvent()

    def getMouseEvent(self):
        buf = self.file.read(3);
        dy,dx = struct.unpack( "bb", buf[1:] )

        current_time = self.get_clock().now()

        odom = Odometry()
        t = TransformStamped()

        # calculate wheel odometry and publish
        dx = float(dx/30000)# not accurate, need further calibration
        dy = float(dy/-30000)#
        vx = dx
        vy = dy
        wz = 0.0
        self.x += dx
        self.y += dy
        self.get_logger().info('x/y: {}/{}'.format(self.x, self.y))
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "mouse"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.quaternion_from_euler(roll = self.th)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "mouse"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = self.quaternion_from_euler(roll = self.th)
        self.odom = odom
        self.t = t
        self.tf_broadcaster.sendTransform(self.t)
        self.publisher_.publish(self.odom)
        
    
    def quaternion_from_euler(self, roll = 0.0, pitch = 0.0, yaw = 0.0):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q = Quaternion()
        q.x = cy * cp * cr + sy * sp * sr
        q.y = cy * cp * sr - sy * sp * cr
        q.z = sy * cp * sr + cy * sp * cr
        q.w = sy * cp * cr - cy * sp * sr

        return q

    def timer_callback(self):
        self.tf_broadcaster.sendTransform(self.t)
        self.publisher_.publish(self.odom)

def main(args=None):
    rclpy.init(args=args)
    
    odom = MouseOdom()
    rclpy.spin(odom)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom.destroy_node()
    rclpy.shutdown()