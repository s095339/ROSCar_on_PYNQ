from pynq import Overlay
from pynq import MMIO
import numpy as np
import time
from time import sleep, time
from .uart_dev import UartAXI


class Serial_IMU(UartAXI):
    def __init__(self, overlay, direction = 0):
        """
        Parameter:
        ----------------
        direction: horizontal:0, veritical:1

        """
        print("Setting the IMU device...")
        super().__init__(overlay = overlay)
        self.direction = direction
        self.Correlation() # 自動校準
        self.data = {}

        #ACC sensitivity--------------------
        self.G=16383.5 #32767/2
        #gyro sensitivity
        self.deg_sec = 16.3835 #32767/2000
        #--------------------
    def Correlation(self):
        #(參考易縉的資料)
        print("IMU correlation ...")
        self.write([0xA5,0x55,0x13,0x0D])#不知道這個要幹嘛
        #自動輸出數據指令
        self.write([0xA5,0x56,0x02,0xFD])
        # 校正 
        
        self.write([0xA5,0x57,0x01,0xFD])#加陀校準
        self.write([0xA5,0x57,0x02,0xFE])#磁力計校準
        # 水平與垂直模式設定
        if self.direction == 0:
            print("set mode to horizontal mode")
            self.write([0xA5,0x5B,0x02,0x02])
        else:
            print("set mode to vertical mode")
            self.write([0xA5,0x5B,0x01,0x01])
    def get_Imu(self):
        """
        Return Data received from IMU GY_25Z
        Data type:
            self.data{
                "ACC":[x,y,z],  unit:g   
                "gyro":[x,y,z], unit:deg/sec
                "YPR":[R,P,Y],  unit:euler angle(deg)
            }
        """
        #init---------------------
        YPR = [0,0,0] 
        gyro=[0,0,0] #角速度
        ACC=[0,0,0] #加速度
        Re_buf = [0x00 for i in range(30)]
        counter=0
        sign=0
        #get data----------------
        for cnt in range(24):
            Re_buf[cnt] = np.int8(self.read(1)[0])
            if(cnt==0 and Re_buf[0]!=0x5A):
                continue
            if(cnt == 23):
                sign=1
        if sign:
            if(Re_buf[0]==0x5A and Re_buf[1]==0x5A):       #检查帧头，帧尾
                ACC[0]=np.int32(Re_buf[4]<<8|Re_buf[5])/self.G  #合成数据，去掉小数点后2位
                ACC[1]=np.int32(Re_buf[6]<<8|Re_buf[7])/self.G
                ACC[2]=np.int32(Re_buf[8]<<8|Re_buf[9])/self.G

                gyro[0]=np.int32(Re_buf[10]<<8|Re_buf[11])/self.deg_sec  #合成数据，去掉小数点后2位
                gyro[1]=np.int32(Re_buf[12]<<8|Re_buf[13])/self.deg_sec
                gyro[2]=np.int32(Re_buf[14]<<8|Re_buf[15])/self.deg_sec

                YPR[0]=np.int(Re_buf[16]<<8|Re_buf[17])/100  #合成数据，去掉小数点后2位
                YPR[1]=np.int(Re_buf[18]<<8|Re_buf[19])/100
                YPR[2]=np.int(Re_buf[20]<<8|Re_buf[21])/100
                #print("frame head = ",Re_buf[0] )
                """
                print("ACC:",end = ",");
                print(ACC[0], end = "\t")
                print(ACC[1], end = "\t")
                print(ACC[2], end = "\t")        

                print(" ,gyro:",end = "\t")
                print(gyro[0], end = "\t")
                print(gyro[1], end = "\t")
                print(gyro[2], end = "\t")    

                print("  ,YPR:",end = "\t")
                print(YPR[2], end = "\t") #显示航向(Y)
                print(YPR[1], end = "\t") #显示俯仰角(P)
                print(YPR[0])                    #显示横滚角(R)     
                """
                self.data["ACC"] = [ACC[0],ACC[1],ACC[2]]
                self.data["gyro"] = [gyro[0],gyro[1],gyro[2]]
                self.data["YPR"] = [YPR[2],YPR[1],YPR[0]]
                sign=0
        
        return self.data

if __name__ == "__main__":
    Imu = Serial_IMU()
