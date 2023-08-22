
# ************************************
#* author: 張耀明
#* 2023/8/19
#* file: Imu.py
#* This file implements the IMU node,
#* node: IMU
#* topic: /Imu/data_raw 
# ************************************

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import threading
import math
import asyncio 
import numpy as np

from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu

import time

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
from pynq.interrupt import Interrupt


from myCar.Car import DCMotor_Car
from myCar.i2c_overlay import I2C_Master
from myIMU.imu import Serial_IMU
import numpy as np

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
        #subscriber
        #self.subscription = self.create_subscription(
        #    String,
        #    "Serial_interrupt",
        #    self.Serialcallback,
        #    10)
        #publisher
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', qos_profile)
        #self.timer_period = 0.1  # seconds
        #self.timer = self.create_timer(self.timer_period, self.timer_callback)


        self.i = 0
        self.previous_time = self.get_clock().now()
        self.start_time = self.get_clock().now()

        #read data
        
        self.cnt = 0
        self.sign = 0
        self.receivedData = {}
        #asyncio handle interrupt
        self.imu_pub()
        #loop = asyncio.get_event_loop() 
        #tasks = [asyncio.ensure_future(self.uart_interrupt())]
        #self.Imu_dev.setupCtrlReg()
        #loop.run_until_complete(asyncio.wait(tasks))
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
    
    #async def uart_interrupt(self):
    def imu_pub(self):

        #node = Node("interrupt_pub")
        #node.interupt_publisher_ = node.create_publisher(String, "Serial_interrupt",10)
        #msg = String()
        #msg.data = "uart_interrupt!"
        YPR = [0,0,0] 
        gyro=[0,0,0] #角速度
        ACC=[0,0,0] #加速度
        self.Imu_dev.setupCtrlReg()
        Re_buf = [0x00 for i in range(30)]
        
        while True:
            
            #await serial_interrupt.wait()# wait for interrupt signal
            #print("interrupt")
            
                #while self.Imu_dev.uart_dev_available():

            Re_buf[self.cnt] = self.Imu_dev.read(1)[0]
            if self.cnt==0 and Re_buf[0]!=0x5A:
                continue
            else:
                self.cnt += 1
            
            if self.cnt == 23 :
                self.sign=1
                self.cnt= 0
            #print("cnt = ",self.cnt)
            if self.sign :
                #self.pub_data()
                if(Re_buf[0]==0x5A and Re_buf[1]==0x5A):       #检查帧头，帧尾
                    #print("yoyoyoy")
                    ACC[0]=((Re_buf[4]<<8|Re_buf[5])/self.Imu_dev.G)  #合成数据，去掉小数点后2位
                    ACC[1]=((Re_buf[6]<<8|Re_buf[7])/self.Imu_dev.G)
                    ACC[2]=((Re_buf[8]<<8|Re_buf[9])/self.Imu_dev.G)
                    gyro[0]=((Re_buf[10]<<8|Re_buf[11])/self.Imu_dev.deg_sec)  #合成数据，去掉小数点后2位
                    gyro[1]=((Re_buf[12]<<8|Re_buf[13])/self.Imu_dev.deg_sec)
                    gyro[2]=((Re_buf[14]<<8|Re_buf[15])/self.Imu_dev.deg_sec)
                    YPR[0]=np.int16(Re_buf[16]<<8|Re_buf[17])/100.#roll  #合成数据，去掉小数点后2位
                    YPR[1]=np.int16(Re_buf[18]<<8|Re_buf[19])/100.#pitch
                    YPR[2]=np.int16(Re_buf[20]<<8|Re_buf[21])/100.#yaw
                    
                    #print("frame head = ",Re_buf[0] )
                    self.receivedData["ACC"] = [ACC[0],ACC[1],ACC[2]]
                    self.receivedData["gyro"] = [gyro[0],gyro[1],gyro[2]]
                    self.receivedData["YPR"] = [YPR[2],YPR[1],YPR[0]]
                    self.get_logger().info(str(self.receivedData["YPR"]) ) 
                    #send topic=
                    imu_data = Imu()
                    current_time = self.get_clock().now()
                    q = self.quaternion_from_euler(
                        self.DEG2RAD(self.receivedData["YPR"][0]),
                        self.DEG2RAD(self.receivedData["YPR"][1]),
                        self.DEG2RAD(self.receivedData["YPR"][2])
                        )
                    imu_data.header.stamp = current_time.to_msg()
                    imu_data.header.frame_id = 'imu'
                    
                    #self.get_logger().debug("Quaternion orientation:")
                    #self.get_logger().info('x:%f ,y:%f ,z:%f ,w:%f' % (q.x,q.y,q.z,q.w))
                    
                    imu_data.orientation.x = q.x
                    imu_data.orientation.y = q.y
                    imu_data.orientation.z = q.z
                    imu_data.orientation.w = q.w
                    # angular_velocity
                    imu_data.angular_velocity.x = self.DEG2RAD(self.receivedData["gyro"][0])
                    imu_data.angular_velocity.y = self.DEG2RAD(self.receivedData["gyro"][1])
                    imu_data.angular_velocity.z = self.DEG2RAD(self.receivedData["gyro"][2])
                    
                    # linear_acceleration
                    imu_data.linear_acceleration.x = self.receivedData["ACC"][0]* 9.8
                    imu_data.linear_acceleration.y = self.receivedData["ACC"][1]* 9.8
                    imu_data.linear_acceleration.z = self.receivedData["ACC"][2]* 9.8
                    self.previous_time = current_time
                    self.publisher_.publish(imu_data)
                self.sign=0
                Re_buf = [0x00 for i in range(30)]


def main(args=None):
    rclpy.init(args=args)
    ov_pth = os.path.join(workspace, 'hardware', 'uart.bit')
    overlay = Overlay(ov_pth,download = False)
    OurImu = Serial_IMU(overlay = overlay, direction = 0)
    IMU_publisher = IMUPublisher(device = OurImu)
    rclpy.spin_once(IMU_publisher)
    
    IMU_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
