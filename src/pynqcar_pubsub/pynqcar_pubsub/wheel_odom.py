import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion

class WheelOdom(Node):

    def __init__(self):
        super().__init__('wheel_odom')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'speed',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # init variables
        self.previous_time = self.get_clock().now()
        # Car spec, assume car goes in x direction
        self.lx = 0.075 # meters
        self.ly = 0.0975 # meters
        self.r = 0.0275 # meters
        self.x = 0.0  # 
        self.y = 0.0  # current orientation
        self.th = 0.0 # 

        # init done message
        self.get_logger().info('Publish /wheel_odom(Odometry)')

    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).to_msg().nanosec / 1E9 # actually from 0.02~0.05 seconds
        V1, V2, V3, V4, _, _, _, _ = msg.data

        odom = Odometry()
        t = TransformStamped()

        # calculate wheel odometry and publish
        vx = float((V1 + V2 + V3 + V4) * self.r / 4) # [m / s]
        vy = float((V1 - V2 + V3 - V4) * self.r / 4) # [m / s]
        wz = float(( -V1 - V2 + V3 + V4) * self.r / 4 / (self.lx + self.ly)) # [rad / s]
        self.x += (vx * math.cos(self.th) - vy * math.sin(self.th)) * dt # [m]
        self.y += (vx * math.sin(self.th) + vy * math.cos(self.th)) * dt # [m]
        self.th += wz * dt  # [rad]
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.quaternion_from_euler(roll = self.th)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = self.quaternion_from_euler(roll = self.th)
        self.tf_broadcaster.sendTransform(t)
        self.publisher_.publish(odom)
        self.previous_time = current_time
    
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

def main(args=None):
    rclpy.init(args=args)
    
    odom = WheelOdom()
    rclpy.spin(odom)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom.destroy_node()
    rclpy.shutdown()