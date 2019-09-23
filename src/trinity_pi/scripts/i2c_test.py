#!/usr/bin/env python
import smbus
import time
import sys
import numpy as np
import struct

bus = smbus.SMBus(1)

while(True):
    try:
        data = bus.read_i2c_block_data(0x03, 0x05, 12)
        print(data)
        print(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3])   
    except:
        pass
    #print("%d, %d, %d" % (struct.unpack('>BL', data[0:4]), struct.unpack('>BL', data[4:8]), struct.unpack('>BL',data[8:12])))
