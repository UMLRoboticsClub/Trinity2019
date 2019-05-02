#include <errno.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include "serial.h"
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h> 
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>

#define REG_GETVAL   0x05
#define REG_ECHO     0x06
#define REG_CLEARVAL 0x07
#define REG_RESET    0x08

using std::string;
using std::cout;
using std::cerr;
using std::endl;

const uint8_t startByte = 170;
const uint8_t endByte = 169;
int num_failures = 0;

union Encoders{
    uint8_t buf[12];
    int32_t enc[3];
} enc;

int fd = 0;

void floosh(int fd){
    usleep(1000);
    tcflush(fd, TCIOFLUSH);
}

void serialInit(const char *tty){
    fd = open(tty, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1){
        ROS_INFO("file bad cannot open");
    }
    set_interface_attribs(fd, B115200, 0);
    set_blocking(fd, 0);
}

void i2c_init(const char* device, int addr) {
    if ((fd = open(device, O_RDWR)) < 0) {
        cerr << "encoders: failed to open i2c bus" << endl;
        exit(1);
    }

    if (ioctl(fd, I2C_SLAVE,addr) < 0) {
        cerr << "encoders: failed to open i2c device" << endl;
        exit(1);
    }
}


//////////
// Set pointer address
//////////
inline void i2c_set_pointer(uint8_t address) {   
    if (write(fd, &address, 1) != 1) {
        cerr << "encoders: write failed" << endl;
    }
}

//////////
// Read n bytes
//////////
bool i2c_read(uint8_t addr, int nbytes) {
    //i2c_set_pointer(addr);

    uint8_t byte = 7;
    uint8_t req = 7;
    bool done = false;
    while(ros::ok() && !done){
        floosh(fd);
        if(write(fd, &req, 1) < 1){
            ROS_INFO("failed to write to serial");
        }
        usleep(1000);
        //ROS_INFO("WTF");
        if(read(fd, &byte, 1) != 1){
            ROS_INFO("yup this one failed too");
        }
        int count = 0;
        //if(num_failures > 25){
        //    return false;
        //}
        /*
        while(ros::ok() && byte != startByte) {

            printf("start byte: %d\n", byte);

            read(fd, &byte, 1);
            num_failures++;
            ROS_INFO("num failures: %d", num_failures);
            if(++count > 13){
                return false;
            }
        }*/

        //ROS_INFO("start byte found");
        int numBytes = 0;
        //printf("start byte: %d\n", byte);
        while(numBytes < 12){
            count ++;
            numBytes += read(fd, enc.buf + numBytes, nbytes-numBytes);
            //ROS_INFO("numGot is: %d", numBytes);
            if(count > 25){
                ROS_INFO("still couldn't get the whole packet");
                continue;
            }
            //usleep(1000);
        }
        if(numBytes != nbytes){
            //ROS_INFO("numBytes is: %d", numBytes);
            ROS_ERROR("read failed");
            continue;
        } 
        read(fd, &byte, 1);
        //printf("end byte: %d\n", byte);
        return true;
        //if(read(fd, &byte, 1), byte != endByte){
            //ROS_INFO("end byte not found!");
            //num_failures++;
            //continue;
        //} else {
            //ROS_INFO("got all data (end byte: %d)", byte);
        //    done = true;
        //}
    }

    /*int ret = read(file, buf, nbytes);
      if(ret != nbytes) {
      fprintf(stderr, "Error reading %i bytes\n",nbytes);
      } else {
      for (n=0;n<nbytes;n++)
    //printf("r_0x%0*x\n", 2, buf[n]);
    return buf;
    }*/
    //return fail;
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "encoders");
    ros::NodeHandle n("~");
    string topic_nameA, topic_nameB, topic_nameC, i2c_device;
    int slave_address, pub_hz;

    topic_nameA = n.param<std::string>("encoder1", "enc1");
    topic_nameB = n.param<std::string>("encoder2", "enc2");
    topic_nameC = n.param<std::string>("encoder3", "enc3");
    //i2c_device = n.param<std::string>("i2c_device", "/dev/i2c-1");
    const char* serialPort = n.param<std::string>("serialPort", "/dev/ttyUSB0").c_str();
    //slave_address = n.param("slave_address", 0x03);
    pub_hz = n.param("pub_hz", 20);
    ROS_INFO("encoders node / encoder1: %s, encoder2: %s, encoder3: %s, i2c_device: %s, slave_address: 0x%x, pub_hz: %d", topic_nameA.c_str(), topic_nameB.c_str(), topic_nameC.c_str(), i2c_device.c_str(), slave_address, pub_hz);
    //i2c_init(i2c_device.c_str(), slave_address);
    serialInit(serialPort);
    ROS_INFO("Opened i2c bus on address %x", slave_address);
    //printf("%s, %d\n", i2c_device, slave_address);

    ros::Publisher pubA = n.advertise<std_msgs::Int32>(topic_nameA, 1);
    ros::Publisher pubB = n.advertise<std_msgs::Int32>(topic_nameB, 1);
    ros::Publisher pubC = n.advertise<std_msgs::Int32>(topic_nameC, 1);
    ros::Rate loop_rate(pub_hz);
    std_msgs::Int32 c;

    //i2c_set_pointer(REG_CLEARVAL);

    ROS_INFO("Publishing encoder values");
    while(ros::ok()){
        i2c_read(REG_GETVAL, 12);
        /*if(num_failures > 25){
            ROS_INFO("sending reset byte");
            num_failures = 0;
            i2c_set_pointer(REG_RESET);
            loop_rate.sleep();
            close(fd);
            i2c_init(i2c_device.c_str(), slave_address);
            loop_rate.sleep();
            continue;
        }*/
        c.data = enc.enc[0];
        pubA.publish(c);
        c.data = enc.enc[1];
        pubB.publish(c);
        c.data = enc.enc[2];
        pubC.publish(c);

        /*for(int i = 0; i < 12; i++){
            printf("%d-", enc.buf[i]);
        }
        printf("\n");*/
        printf("%d, %d, %d\n", enc.enc[0], enc.enc[1], enc.enc[2]);

        ros::spinOnce();
        loop_rate.sleep();
    }
    close(fd);
}
