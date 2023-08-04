# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import threading
import math

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

# add python path
import os
import sys
from pathlib import Path
workspace = Path(__file__).parents[6] # 3-level up from __file__
sys.path.append('/usr/local/share/pynq-venv/lib/python3.10/site-packages')
sys.path.append(os.path.join(workspace, 'include'))
sys.path.append(os.path.join(workspace, 'hardware'))

import pynq
from pynq import Overlay
from pynq import MMIO
from myCar.Car import DCMotor_Car
from myCar.i2c_overlay import I2C_Master

# Car spec, assume car goes in y direction
lx = 0.195 # meters
ly = 0.15 # meters
r = 0.0275 # meters


class DriverCmdSubscriber(Node):

    def __init__(self, device):
        self.Car = device
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        w1 = (vx+vy-(lx+ly)*wz)/r # [rad/s], wheel rear left
        w2 = (vx-vy-(lx+ly)*wz)/r # [rad/s], wheel front left
        w3 = (vx+vy+(lx+ly)*wz)/r # [rad/s], wheel front right
        w4 = (vx-vy+(lx+ly)*wz)/r # [rad/s], wheel rear right

        # quantize speed [rad/s] (max +- 11.2) to -255 to 255
        q1 = int(w1/11.2*255)
        q2 = int(w2/11.2*255)
        q3 = int(w3/11.2*255)
        q4 = int(w4/11.2*255)

        # for debug
        self.get_logger().info('q1: "%d"' % q1)
        self.get_logger().info('q2: "%d"' % q2)
        self.get_logger().info('q3: "%d"' % q3)
        self.get_logger().info('q4: "%d"' % q4)
        
        self.Car.M1.speed(q1)
        self.Car.M2.speed(q2)
        self.Car.M3.speed(q3)
        self.Car.M4.speed(q4)

class IMUPublisher(Node):

    def __init__(self,overlay):
        super().__init__('IMUPublisher')
        self.overlay = overlay
        self.publisher_ = self.create_publisher(String, 'speed_cmd', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = str(self.overlay.encoder_0.read(0x04))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

class WheelOdomPublisher(Node):

    def __init__(self,overlay):
        super().__init__('wheel_odometry_publisher')
        self.overlay = overlay
        self.publisher_ = self.create_publisher(Odometry, 'wheel_odom', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.previous_time = self.get_clock().now()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.pre_M1 = 0.0
        self.pre_M2 = 0.0
        self.pre_M3 = 0.0
        self.pre_M4 = 0.0

    def timer_callback(self):
        odom = Odometry()
        t = TransformStamped()
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).to_msg().nanosec / 1E9

        # M1 ~ M4 is current encoder values
        M1 = self.uint32toint(self.overlay.encoder_0.read(0x00)) # wheel rear left [encoder ticks]
        M2 = self.uint32toint(self.overlay.encoder_0.read(0x04)) # wheel front left [encoder ticks]
        M3 = self.uint32toint(self.overlay.encoder_0.read(0x08)) # wheel front right [encoder ticks]
        M4 = self.uint32toint(self.overlay.encoder_0.read(0x0c)) # wheel rear right [encoder ticks]
        
        V1 = (M1 - self.pre_M1) / dt / 48 / 90 * 2 * math.pi # [rad / t], 48 * 90 tick per rotation, gear ratio 1:90 
        V2 = (M2 - self.pre_M2) / dt / 48 / 90 * 2 * math.pi # [rad / t]
        V3 = (M3 - self.pre_M3) / dt / 48 / 90 * 2 * math.pi # [rad / t]
        V4 = (M4 - self.pre_M4) / dt / 48 / 90 * 2 * math.pi # [rad / t]

        self.pre_M1 = M1
        self.pre_M2 = M2
        self.pre_M3 = M3
        self.pre_M4 = M4

        vx = float((V1 + V2 + V3 + V4) * r / 4) # [m / s]
        vy = float((V1 - V2 + V3 - V4) * r / 4) # [m / s]
        wz = float(( -V1 - V2 + V3 + V4) * r / 4 / (lx + ly)) # [rad / s]

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

    def uint32toint(self, value):
        if (value>= 2**31): # negative value
            value-= 2**32
        return value
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
    print("workspace = ",workspace)
    rclpy.init(args=args)
    ov_pth = os.path.join(workspace, 'hardware', 'encoder.bit')
    overlay = Overlay(ov_pth)
    #print("................b................")
    i2c_dev = I2C_Master(overlay)
    #print("................c................")
    OurCar = DCMotor_Car(i2c_dev)
    #print('................d................')
    OurCar.run("stop")
    minimal_subscriber = DriverCmdSubscriber(device=OurCar)
    IMU_publisher = IMUPublisher(overlay = overlay)
    odom_publisher = WheelOdomPublisher(overlay = i2c_dev.overlay)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(minimal_subscriber)
    executor.add_node(IMU_publisher)
    executor.add_node(odom_publisher)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    #rclpy.spin(minimal_subscriber)
    rate = minimal_subscriber.create_rate(2)
    try:
        while rclpy.ok():
            print('Help me body, you are my only hope')
            rate.sleep()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    executor_thread.join()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #minimal_subscriber.destroy_node()
    #rclpy.shutdown()


if __name__ == '__main__':
    main()
