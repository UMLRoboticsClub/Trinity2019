#!/usr/bin/env python
import smbus
import time
import sys
import rospy
from geometry_msgs.msg import Vector3

class FXAS21002C:
    # range: 
    #   0 = 250 DPS
    #   1 = 500 DPS
    #   2 = 1000 DPS
    #   3 = 2000 DPS
    def __init__(self, range):
        self.ADDR = 0x21 
        self.ID = 0xD7
        self.SENSORS_DPS_TO_RADS = 0.017453293
        self.RANGE_CRTL_REG = [0x03, 0x02, 0x01, 0x00]
        self.RANGE_SENSITIVITY = [0.0078125, 0.015625, 0.03125, 0.0625]
        self.STATUS = 0x00
        self.OUT_X_MSB = 0x01
        self.OUT_X_LSB = 0x02
        self.OUT_Y_MSB = 0x03
        self.OUT_Y_LSB = 0x04
        self.OUT_Z_MSB = 0x05
        self.OUT_Z_LSB = 0x06
        self.WHO_AM_I = 0x0C
        self.CRTL_REG0 = 0x0D
        self.CRTL_REG1 = 0x13
        self.CRTL_REG2 = 0x14
        self.bus = smbus.SMBus(1)
        self.range = range
        self.vel = Vector3()
        self.offset = Vector3()
        self.calibrated = False
        data = self.bus.read_byte_data(self.ADDR, self.WHO_AM_I)
        if(data is not self.ID):
            rospy.logerr("whoami response incorrect :( (got: {0} instead of {1}".format(hex(data), hex(self.ID)))
            exit(1)
        self.bus.write_byte_data(self.ADDR, self.CRTL_REG1, 0x00)
        try:
            self.bus.write_byte_data(self.ADDR, self.CRTL_REG1, 1<<6)
        except:
            pass
        
        self.bus.write_byte_data(self.ADDR, self.CRTL_REG0, self.RANGE_CRTL_REG[range])
        self.bus.write_byte_data(self.ADDR, self.CRTL_REG1, 0x0E)

        self.calibrate()
        rospy.loginfo("finished initializing gyro")
        time.sleep(0.1)

    def calibrate(self):

        for i in range(100):
            self.update()
            self.offset.x += self.vel.x
            self.offset.y += self.vel.y
            self.offset.z += self.vel.z
            rospy.loginfo(self.vel.z)
            time.sleep(0.05)

        self.offset.x /= 100
        self.offset.y /= 100
        self.offset.z /= 100
        self.calibrated = True

    def update(self):
        data = self.bus.read_i2c_block_data(self.ADDR, self.STATUS | 0x80, 7)
        #rospy.loginfo(hex(self.STATUS | 0x80))
        #rospy.loginfo(data)
        raw = [(data[1] << 8) | data[2], (data[3] << 8) | data[4], (data[5] << 8) | data[6]]
        #rospy.loginfo(raw)
        self.vel.x = raw[0] * self.RANGE_SENSITIVITY[self.range]
        self.vel.y = raw[1] * self.RANGE_SENSITIVITY[self.range]
        self.vel.z = raw[2] * self.RANGE_SENSITIVITY[self.range]

        self.vel.x *= self.SENSORS_DPS_TO_RADS  
        self.vel.y *= self.SENSORS_DPS_TO_RADS
        self.vel.z *= self.SENSORS_DPS_TO_RADS

        if(self.calibrated):
            rospy.loginfo("velocity before: {0}, {1}, {2}".format(self.vel.x, self.vel.y, self.vel.z))
            self.vel.x -= self.offset.x
            self.vel.y -= self.offset.y
            self.vel.z -= self.offset.z
            rospy.loginfo("subtracted offset: {0}, {1}, {2}".format(self.offset.x, self.offset.y, self.offset.z))
    
    def close(self):
        self.bus.close()

def main():
    rospy.init_node('gyro')
    pub_hz = rospy.get_param("pub_hz", 20)
    gyro_topic = rospy.get_param("gyro_topic", "/gyro")

    rate = rospy.Rate(pub_hz)
    pub = rospy.Publisher(gyro_topic, Vector3, queue_size=10)
    gyro = FXAS21002C(0)

    while not rospy.is_shutdown():
        gyro.update()
        rospy.loginfo("x: {0}, y: {1}, z: {2}".format(gyro.vel.x, gyro.vel.y, gyro.vel.z))
        pub.publish(gyro.vel)
        rate.sleep()
    
    gyro.close()



if __name__ == '__main__':
    main()
