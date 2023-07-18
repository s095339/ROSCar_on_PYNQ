
from .i2c_util import *
from math import floor
class DCMotor_Car:
    def __init__(self,i2c_object):
        self.M1 = DCMotor(i2c_object,8,9)
        self.M2 = DCMotor(i2c_object,10,11)
        self.M3 = DCMotor(i2c_object,15,14)
        self.M4 = DCMotor(i2c_object,13,12)
        self.motor_i2c_ctrl = i2c_object
        self.motor_i2c_ctrl.set_slave_addr("Car",0x60)
        self.motor_i2c_ctrl.set_default_slave_device("Car")
        self.initial_setting()
        print("initial setting completed!")
        
    def initial_setting(self):
        write8(self.motor_i2c_ctrl, PCA9685_MODE1,0x0)
        freq = 50.
        freq = freq*0.9
        prescaleval = 25000000.
        prescaleval /= 4096
        prescaleval /= freq
        prescaleval -= 1
        prescale = floor(prescaleval+0.5)
        print("reading data from motordriver...")
        oldmode = read8(self.motor_i2c_ctrl,PCA9685_MODE1)
        newmode =(oldmode&0x7F) | 0x10
        print("newmode=",newmode)
        print("oldmode=",oldmode)
        i=0
        while i<50000:
            i=i+1
        write8(self.motor_i2c_ctrl,PCA9685_MODE1, newmode) # go to sleep
        write8(self.motor_i2c_ctrl,PCA9685_PRESCALE, prescale) # set the prescaler
        write8(self.motor_i2c_ctrl,PCA9685_MODE1, oldmode)
        #delay(5);
        i =0
        while i<5000:
            i=i+1
        write8(self.motor_i2c_ctrl,PCA9685_MODE1, oldmode | 0xa1)
        for i in range(16):
            setPWM(self.motor_i2c_ctrl,i,0,0)
    
    def run(self,direction,speed=200):
      
        self.motor_i2c_ctrl.set_default_slave_device("Car")
        if direction == "forward":
            self.M1.forward(speed)
            self.M2.forward(speed)
            self.M3.forward(speed)
            self.M4.forward(speed)
        elif direction == "backward":
            self.M1.backward(speed)
            self.M2.backward(speed)
            self.M3.backward(speed)
            self.M4.backward(speed)
        elif direction == "left":
            pass
        elif direction == "right":
            pass
        else:
            self.M1.stop()
            self.M2.stop()
            self.M3.stop()
            self.M4.stop()
class DCMotor:
    def __init__(self, motor_i2c_ctrl,in1,in2):
        self.In1 = in1;
        self.In2 = in2;
        self.motor_i2c_ctrl = motor_i2c_ctrl
    def forward(self, speed = 200):
        pin = self.In2
        setPWM(self.motor_i2c_ctrl,pin,0,0);
        pin = self.In1
        _speed = speed*16
        if _speed>4095 :
            setPWM(self.motor_i2c_ctrl,pin,4096,0)
        else:
            setPWM(self.motor_i2c_ctrl,pin,0,_speed)
    def backward(self,speed = 200):
        #pin = self.In
        pin = self.In1
        setPWM(self.motor_i2c_ctrl,pin,0,0)
 
        pin = self.In2
        _speed = speed*16;
        if _speed > 4095:
            setPWM(self.motor_i2c_ctrl, pin, 4096, 0);
        else:
            setPWM(self.motor_i2c_ctrl, pin, 0, _speed);
    def stop(self):
        
        pin = self.In2
        setPWM(self.motor_i2c_ctrl,pin,0,0);
        pin = self.In1
        setPWM(self.motor_i2c_ctrl,pin,0,0);

#vector<int> dd;
