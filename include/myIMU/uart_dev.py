
from pynq import Overlay
from pynq import MMIO
import numpy as np
import time
from time import sleep, time

address = 0xa0010000

# HARDWARE CONSTANTS
RX_FIFO = 0x00
TX_FIFO = 0x04

# Status Reg
STAT_REG = 0x08
RX_VALID = 0
RX_FULL = 1
TX_EMPTY = 2
TX_FULL = 3
IS_INTR = 4
OVERRUN_ERR = 5
FRAME_ERR = 6
PARITY_ERR = 7

# Ctrl Reg
CTRL_REG = 0x0C
RST_TX = 0
RST_RX = 1
INTR_EN = 4

# Offset Settings
XGPIO_DATA_OFFSET = 0x0
XGPIO_TRI_OFFSET = 0x4

class UartAXI:
    def __init__(self, overlay , address = None):
        # Setup axi core
        #self.uart = MMIO(address, 0x10000, debug=False)
        print("Setting the Serial Uart Device")
        self.address = address
        self.overlay = overlay
        self.uart = overlay.axi_uartlite_0
    def setupCtrlReg(self):
        # Reset FIFOs, enable interrupts
        self.uart.write(CTRL_REG, 1 << RST_TX | 1 << RST_RX | 1 << INTR_EN )
        sleep(1)
        self.uart.write(CTRL_REG, 0| 1 << INTR_EN)
        sleep(1)
    def read(self, count, timeout=10):
        # status = currentStatus(uart) bad idea
        buf = []
        stop_time = time() + timeout
        for i in range(count):
            # Wait till RX fifo has valid data, stop waiting if timeoutpasses
            while (not (self.uart.read(STAT_REG) & 1 << RX_VALID)) and (time() < stop_time):
                pass
            if time() >= stop_time:
                break
            buf.append(self.uart.read(RX_FIFO))
        return buf
    def write(self, buf, timeout=10):
        # Write bytes via UART
        stop_time = time() + timeout
        wr_count = 0
        for i in buf:
            # Wait while TX FIFO is Full, stop waiting if timeout passes
            while (self.uart.read(STAT_REG) & 1 << TX_FULL) and (time() < stop_time):
                pass
            # Check timeout
            if time() > stop_time:
                break
            self.uart.write(TX_FIFO, i)
            wr_count += 1
        return wr_count
    def uart_dev_available(self):
        return (self.uart.read(STAT_REG) & 1 << RX_VALID)
if __name__ == "__main__":
    uart = UartAXI(address,ol)
    # Setup AXI UART register
    uart.setupCtrlReg()


