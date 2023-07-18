from pynq import Overlay
from pynq import MMIO
import time
from time import sleep, time

import pynq.lib as lib
import cffi
ffi = cffi.FFI()
class I2C_Master:
    
    overlay = None
    i2c = None
    def init_hw(self, overlay):
        self.overlay = overlay
        #self.overlay = Overlay(filepath,ignore_version=True)
        print("Overlay Design:")
        print(self.overlay)
        self.overlay.download()
        
    def __init__(self, bitstream_file):
        # Initializing bitstream
        print("initializing hardware....")
        self.init_hw(bitstream_file)
        print("init hardware finished")
        self.i2c = lib.AxiIIC(self.overlay.ip_dict['axi_iic_0'])
        self.slave_addr_dict = {
            "default":0x00,
        }
        self.default_slave = "";
        
    def set_slave_addr(self, device:str ,addr:hex):
        """
        Set Slave device to support the master communicating with multiple i2c device
        
        Parameter
        -------------
        device: str, device's name
        addr: hex,   device's addr
        
        Each device and the related addr are stored in a dict
        self.slave_addr_dict[device] = addr
        
        """
        self.slave_addr_dict[device] = addr
    def set_default_slave_device(self, device:str):
        if device not in self.slave_addr_dict.keys():
            raise ValueError("Device Name Not Found! ")
        else:
            self.default_slave = device
    def send(self, data:list ,device = ""):
        slave_device = ""
        if device == "":
            assert self.default_slave != ""
            slave_device = self.default_slave
        else:
            slave_device = device
        
        slave_addr = self.slave_addr_dict[slave_device]
        #buf = []
        #buf.append(self.slave_addr_dict[slave_device])
        #for d in data:
        #    buf.append(d)
        self.i2c.send(slave_addr, data, len(data), 0)
                
    def receive(self, length:int, device = ""):
        cdata_ptr = ffi.new("unsigned char *")
        slave_device = ""
        if device == "":
            assert self.default_slave != ""
            slave_device = self.default_slave
        else:
            slave_device = device
        slave_addr = self.slave_addr_dict[slave_device]
        re = self.i2c.receive(slave_addr, cdata_ptr, length, option=0)
     
        return cdata_ptr[0]