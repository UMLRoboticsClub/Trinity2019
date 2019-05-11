#!/bin/bash
scp ./build/bootloader/bootloader.bin ./build/i2c-slave.bin ./build/partitions_singleapp.bin pi@192.168.1.100:~/trinity_catkin_ws/src/microcontroller_code
