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
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import threading
import math
import asyncio 
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

from sensor_msgs.msg import Imu

# add python path
import os
import sys
from pathlib import Path
workspace = Path(__file__).parents[3] # 3-level up from __file__
sys.path.append('/usr/local/share/pynq-venv/lib/python3.10/site-packages')
sys.path.append(os.path.join(workspace, 'include'))
sys.path.append(os.path.join(workspace, 'hardware'))

import pynq
from pynq import Overlay
from pynq import MMIO
from pynq.interrupt import Interrupt


from myCar.Car import DCMotor_Car
from myCar.i2c_overlay import I2C_Master
from myIMU.imu import Serial_IMU
import numpy as np

# Car spec, assume car goes in x direction
lx = 0.075 # meters
ly = 0.0975 # meters

r = 0.0275 # meters

class IMUPublisher(Node):

    def __init__(self, device):
        super().__init__('IMUPublisher')
        self.Imu_dev = device
        """
        baut rate: 1/115200 *10bits* NumOfData(24) =  0.02083333s per Data.
        """

        #不知道這段要幹嘛反正放一下----------------------------------
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        #----------------------------------------------------------
        
        #publisher
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', qos_profile)
        #timer_period = 1/115200 * 10 * 24 = 0.002833

        #callback function
        timer_period = 0.02 #20ms
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.previous_time = self.get_clock().now()
        self.pub_data()
        #interrupt control
        #self.loop = asyncio.get_event_loop() 
        #print(device.uart._interrupts)
        #self.uart_interrupt = Interrupt('axi_uartlite_0/interrupt')
        #asyncio.run(self.pub_data())
        #self.loop.run_until_complete(self.pub_data())
    def DEG2RAD(self, degree):
        return degree *  math.pi/180.
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
    #async def pub_data(self):
    def pub_data(self):
        YPR = [0,0,0] 
        gyro=[0,0,0] #角速度
        ACC=[0,0,0] #加速度
        Re_buf = [0x00 for i in range(30)]
        sign=0
        cnt=0
        receivedData = {}
        #clean Rx FIFO-------
        self.Imu_dev.setupCtrlReg()
        #--------------------
        while True:
            #if self.Imu_dev.uart_dev_available():
            #print("waiting for interrupt")
            #asyncio.run(self.uart_interrupt.wait()) # wait for interrupt
            imu_data = Imu()
            current_time = self.get_clock().now()
            #------------------------------------
            #receivedData = self.Imu_dev.get_Imu()
            
            Re_buf[cnt] = np.int8(self.Imu_dev.read(1)[0])
            if(cnt==0 and Re_buf[0]!=0x5A):
                continue
            cnt = cnt+1
            if(cnt == 23):
                sign=1
            
            
            if sign:
                if(Re_buf[0]==0x5A and Re_buf[1]==0x5A):       #检查帧头，帧尾
                    ACC[0]=np.int32((Re_buf[4]<<8|Re_buf[5])/self.Imu_dev.G)  #合成数据，去掉小数点后2位
                    ACC[1]=np.int32((Re_buf[6]<<8|Re_buf[7])/self.Imu_dev.G)
                    ACC[2]=np.int32((Re_buf[8]<<8|Re_buf[9])/self.Imu_dev.G)

                    gyro[0]=np.int32((Re_buf[10]<<8|Re_buf[11])/self.Imu_dev.deg_sec)  #合成数据，去掉小数点后2位
                    gyro[1]=np.int32((Re_buf[12]<<8|Re_buf[13])/self.Imu_dev.deg_sec)
                    gyro[2]=np.int32((Re_buf[14]<<8|Re_buf[15])/self.Imu_dev.deg_sec)

                    YPR[0]=(Re_buf[16]<<8|Re_buf[17])/100#roll  #合成数据，去掉小数点后2位
                    YPR[1]=(Re_buf[18]<<8|Re_buf[19])/100#pitch
                    YPR[2]=(Re_buf[20]<<8|Re_buf[21])/100#yaw
                    
                    print("  ,YPR:",end = "\t")
                    print(YPR[2], end = "\t") #显示航向
                    print(YPR[1], end = "\t") #显示俯仰角
                    print(YPR[0])                    #显示横滚角     
                    
                    

                    receivedData["ACC"] = [ACC[0],ACC[1],ACC[2]]
                    receivedData["gyro"] = [gyro[0],gyro[1],gyro[2]]
                    receivedData["YPR"] = [YPR[2],YPR[1],YPR[0]]
            #------------------------------------
            #Data send from IMU GY_25Z through UART interface========

            #Data processing
            #Quaternion orientation (transformed from Euler angles)
                    q = self.quaternion_from_euler(
                        self.DEG2RAD(receivedData["YPR"][0]),
                        self.DEG2RAD(receivedData["YPR"][1]),
                        self.DEG2RAD(receivedData["YPR"][2])
                        )
                    # for debug
                    
                    imu_data.header.stamp = current_time.to_msg()
                    imu_data.header.frame_id = 'imu'
                    self.i+=1
                    
                    imu_data.orientation.x = q.x
                    imu_data.orientation.y = q.y
                    imu_data.orientation.z = q.z
                    imu_data.orientation.w = q.w
                    # angular_velocity
                    imu_data.angular_velocity.x = self.DEG2RAD(receivedData["gyro"][0])
                    imu_data.angular_velocity.y = self.DEG2RAD(receivedData["gyro"][1])
                    imu_data.angular_velocity.z = self.DEG2RAD(receivedData["gyro"][2])
                    
                    # linear_acceleration
                    imu_data.linear_acceleration.x = receivedData["ACC"][0]* 9.8
                    imu_data.linear_acceleration.y = receivedData["ACC"][1]* 9.8
                    imu_data.linear_acceleration.z = receivedData["ACC"][2]* 9.8
                    self.previous_time = current_time
                    self.publisher_.publish(imu_data)
                sign=0
                cnt = 0
                Re_buf = [0x00 for i in range(30)]
            #else:
            #    print("")

class PIDspeedControl(Node):
    def __init__(self,overlay, PID = True):
        super().__init__('PIDspeedControl')

        self.Kp = 28.0
        self.Ki = 5.5
        self.Kd = 0.0
        self.PID = PID
        self.get_logger().info('Publish /speed(Float64MultiArray) with format [V1, V2, V3, V4, t1, t2, t3, t4], V = measured speed, t = target speed')
        if not PID:
            self.get_logger().info('Subscribe /cmd_vel(Twist) for open loop speed control')
        else:
            self.get_logger().info('Subscribe /cmd_vel(Twist) for PID speed control')
            self.get_logger().info('Kp/Ki/Kd: {}/{}/{}'.format(self.Kp,self.Ki,self.Kd))

        # call overlay
        self.overlay = overlay
        i2c_dev = I2C_Master(overlay)
        car = DCMotor_Car(i2c_dev)
        self.Car = car
        car.run("stop")

        # create publisher
        timer_period = 0.04  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'speed', 10)
        
        # create subscriber
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # init variables
        self.previous_time = self.get_clock().now()
        self.pre_M1 = 0.0
        self.pre_M2 = 0.0
        self.pre_M3 = 0.0
        self.pre_M4 = 0.0
        self.t1 = 0.0 # target speed of M1, [rad/s]
        self.t2 = 0.0 # target speed of M2, [rad/s]
        self.t3 = 0.0 # target speed of M3, [rad/s]
        self.t4 = 0.0 # target speed of M4, [rad/s]
        self.pre_e1 = 0.0 # e[k-1]
        self.pre_e2 = 0.0 #
        self.pre_e3 = 0.0 #
        self.pre_e4 = 0.0 # 
        self.pre2_e1 = 0.0 # e[k-2]
        self.pre2_e2 = 0.0 #
        self.pre2_e3 = 0.0 #
        self.pre2_e4 = 0.0 #
        self.w1 = 0.0
        self.w2 = 0.0
        self.w3 = 0.0
        self.w4 = 0.0
        # test
        self.av1 = 0.0
        self.av2 = 0.0
        self.av3 = 0.0
        self.av4 = 0.0

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.previous_time).to_msg().nanosec / 1E9 # actually from 0.02~0.05 seconds

        # M1 ~ M4 is current encoder values
        M1 = self.uint32toint(self.overlay.encoder_0.read(0x00)) # wheel rear left [encoder ticks]
        M2 = self.uint32toint(self.overlay.encoder_0.read(0x04)) # wheel front left [encoder ticks]
        M3 = self.uint32toint(self.overlay.encoder_0.read(0x08)) # wheel front right [encoder ticks]
        M4 = self.uint32toint(self.overlay.encoder_0.read(0x0c)) # wheel rear right [encoder ticks]

        V1 = (M1 - self.pre_M1) / dt / 4317.4 * 2 * math.pi # [rad / s], 48 * 90 tick per rotation, gear ratio 1:90 
        V2 = (M2 - self.pre_M2) / dt / 4323.3 * 2 * math.pi # [rad / s]
        V3 = (M3 - self.pre_M3) / dt / 4317.3 * 2 * math.pi # [rad / s]
        V4 = (M4 - self.pre_M4) / dt / 4310.0 * 2 * math.pi # [rad / s]

        # measure
        self.av1 = 0.5 * V1 + 0.5 * self.av1
        self.av2 = 0.5 * V2 + 0.5 * self.av2
        self.av3 = 0.5 * V3 + 0.5 * self.av3
        self.av4 = 0.5 * V4 + 0.5 * self.av4
        #self.get_logger().info('velocity:{}/{}/{}/{}'.format(V1, V2, V3, V4))
        #self.get_logger().info('average velocity:{}/{}/{}/{}'.format(self.av1, self.av2, self.av3, self.av4))
        

        self.pre_M1 = M1
        self.pre_M2 = M2
        self.pre_M3 = M3
        self.pre_M4 = M4

        # publish wheel speed and PID target speed
        msg = Float64MultiArray()
        msg.data = [V1, V2, V3, V4, self.t1, self.t2, self.t3, self.t4]
        self.publisher_.publish(msg)

        self.previous_time = current_time

        if (not self.PID):
            self.Car.M1.speed(int(self.t1 / 4.5 * 255))
            self.Car.M2.speed(int(self.t2 / 4.5 * 255))
            self.Car.M3.speed(int(self.t3 / 4.5 * 255))
            self.Car.M4.speed(int(self.t4 / 4.5 * 255))
            return
        
        # Incremental PID speed control
        e1 = self.t1 - V1 # PID error, [rad/s]
        e2 = self.t2 - V2 # PID error, [rad/s]
        e3 = self.t3 - V3 # PID error, [rad/s]
        e4 = self.t4 - V4 # PID error, [rad/s]

        self.w1 += self.Kp * (e1 - self.pre_e1) + self.Ki * e1 + self.Kd * (e1 - 2 * self.pre_e1 + self.pre2_e1)
        self.w2 += self.Kp * (e2 - self.pre_e2) + self.Ki * e2 + self.Kd * (e2 - 2 * self.pre_e2 + self.pre2_e2)
        self.w3 += self.Kp * (e3 - self.pre_e3) + self.Ki * e3 + self.Kd * (e3 - 2 * self.pre_e3 + self.pre2_e3)
        self.w4 += self.Kp * (e4 - self.pre_e4) + self.Ki * e4 + self.Kd * (e4 - 2 * self.pre_e4 + self.pre2_e4)

        self.pre2_e1 = self.pre_e1
        self.pre2_e2 = self.pre_e2
        self.pre2_e3 = self.pre_e3
        self.pre2_e4 = self.pre_e4
        self.pre_e1 = e1
        self.pre_e2 = e2
        self.pre_e3 = e3
        self.pre_e4 = e4

        # clip pwm value to -255~255
        self.w1 = np.clip(self.w1, -255, 255)
        self.w2 = np.clip(self.w2, -255, 255)
        self.w3 = np.clip(self.w3, -255, 255)
        self.w4 = np.clip(self.w4, -255, 255)
        
        #send speed
        self.Car.M1.speed(int(self.w1))
        self.Car.M2.speed(int(self.w2))
        self.Car.M3.speed(int(self.w3))
        self.Car.M4.speed(int(self.w4))

    def uint32toint(self, value):
        if (value>= 2**31): # negative value
            value-= 2**32
        return value

    def listener_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self.t1 = (vx+vy-(lx+ly)*wz)/r # target speed [rad/s], wheel rear left
        self.t2 = (vx-vy-(lx+ly)*wz)/r # target speed [rad/s], wheel front left
        self.t3 = (vx+vy+(lx+ly)*wz)/r # target speed [rad/s], wheel front right
        self.t4 = (vx-vy+(lx+ly)*wz)/r # target speed [rad/s], wheel rear right

def main(args=None):
    print("workspace = ",workspace)
    rclpy.init(args=args)
    ov_pth = os.path.join(workspace, 'hardware', 'uart.bit')
    overlay = Overlay(ov_pth)
    # For DC Motor setup(I2c interface) 
    i2c_dev = I2C_Master(overlay)
    OurCar = DCMotor_Car(i2c_dev)
    OurCar.run("stop")
    # For IMU setup(Uart interface)
    OurImu = Serial_IMU(overlay = overlay, direction = 0)
    pid = PIDspeedControl(overlay = overlay, PID = True)

    #Three nodes ------------------------------------------
    IMU_publisher = IMUPublisher(device = OurImu)
    #------------------------------------------------------
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(IMU_publisher)
    executor.add_node(pid)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = pid.create_rate(2)
    try:
        while rclpy.ok():
            #print('Help me body, you are my only hope')
            rate.sleep()
    except KeyboardInterrupt:
        
        pass
    rclpy.shutdown()
    executor_thread.join()
    OurCar.run("stop")
    executor.shutdown()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    


if __name__ == '__main__':
    main()
