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

from std_msgs.msg import String
import pynq
from pynq import Overlay
from pynq import MMIO
from myCar.Car import DCMotor_Car
from myCar.i2c_overlay import I2C_Master

class DriverCmdSubscriber(Node):

    def __init__(self, device):
        self.Car = device
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'drive_cmd',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == "forward":
            self.Car.run("forward")
        elif msg.data == "backward":
            self.Car.run("backward")
        else:
            self.Car.run("stop")

class SpeedPublisher(Node):

    def __init__(self,overlay):
        super().__init__('minimal_publisher')
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


def main(args=None):
    rclpy.init(args=args)
    ov_pth = "./hardware/debug.bit"
    overlay = Overlay(ov_pth)
    #print("................b................")
    i2c_dev = I2C_Master(overlay)
    #print("................c................")
    OurCar = DCMotor_Car(i2c_dev)
    #print('................d................')
    OurCar.run("stop")
    minimal_subscriber = DriverCmdSubscriber(device=OurCar)
    speed_publisher = SpeedPublisher(overlay = overlay)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(minimal_subscriber)
    executor.add_node(speed_publisher)
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
