
"""
Rewrite the DCMotor Driver from Arduino Driver 
"""
from pynq.overlays.base import BaseOverlay
from pynq.lib.pmod import pmod_iic




_i2caddr =  0x60

FORWARD  = 1
BACKWARD  = 2
BRAKE = 3
RELEASE = 4

PCA9685_SUBADR1 = 0x2
PCA9685_SUBADR2 = 0x3
PCA9685_SUBADR3 = 0x4

PCA9685_MODE1 = 0x0
PCA9685_PRESCALE = 0xFE

LED0_ON_L = 0x6
LED0_ON_H = 0x7
LED0_OFF_L = 0x8
LED0_OFF_H = 0x9

ALLLED_ON_L = 0xFA
ALLLED_ON_H = 0xFB
ALLLED_OFF_L = 0xFC
ALLLED_OFF_H = 0xFD

def setPWM(motor_i2c_ctrl,num, on ,off):
    """
    
    """
    motor_i2c_ctrl.send([LED0_ON_L+4*num,on&0xFF,on>>8,off&0xFF,off>>8])
    #motor_i2c_ctrl.send([on,on>>8,off,off>>8])
    #motor_i2c_ctrl.send([on>>8])
    #motor_i2c_ctrl.send([off])
    #motor_i2c_ctrl.send([off>>8])
    return

def read8(motor_i2c_ctrl,addr):
    motor_i2c_ctrl.send([addr])
    re = motor_i2c_ctrl.receive(1)
  
    return re 

def write8(motor_i2c_ctrl,addr,d):
    motor_i2c_ctrl.send([addr,d]);
    return
    
    
