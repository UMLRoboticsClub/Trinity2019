#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <cstdint>
#include <iostream>
#include <thread>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

static union {
    uint8_t buf[12];
    int32_t counter[4];
};

int main(int argc, char** argv){
    ros::init(argc, argv, "encoders");
    ros::NodeHandle n("~");
    string topic_nameA, topic_nameB, topic_nameC, i2c_device;
    int slave_address, file, pub_hz;
    char* buffer;

    n.getParam("encoder1", topic_nameA);
    n.getParam("encoder2", topic_nameB);
    n.getParam("encoder3", topic_nameC);
    n.getParam("i2c_device", i2c_device);
    n.getParam("slave_address", slave_address);
    n.getParam("pub_hz", pub_hz);
    ROS_INFO("encoders node / encoder1: %s, encoder2: %s, encoder3: %s, i2c_device: %s, slave_address: 0x%x, pub_hz: %d", topic_nameA.c_str(), topic_nameB.c_str(), topic_nameC.c_str(), i2c_device.c_str(), slave_address, pub_hz);

    //printf("%s, %d\n", i2c_device, slave_address);
    
    int fd;
    if((fd = open(device, O_RDWR)) < 0){
        cerr << "Unable to open I2C bus" << endl;
    }
    if(ioctl(fd, I2C_SLAVE, slave_address) < 0){
        cerr << "Unable to connect to I2C device" << endl;
    }

    ROS_INFO("Opened i2c bus on address %x", slave_address);

    ros::Publisher pubA = n.advertise<std_msgs::Int32>(topic_nameA, 1000);
    ros::Publisher pubB = n.advertise<std_msgs::Int32>(topic_nameB, 1000);
    ros::Publisher pubC = n.advertise<std_msgs::Int32>(topic_nameC, 1000);
    ros::Rate loop_rate(pub_hz);
    std_msgs::Int32 c;

    ROS_INFO("Publishing encoder values");
    while(ros::ok()){

        const int numBytes = 12;
        const uint8_t addr = 0x05;

        if(write(fd, &addr, 1) != 1){
            cerr << "write failed" << endl;
            continue;
        }

        uint8_t startByte = 170;
        uint8_t endByte = 169;

        uint8_t byte;
        while(read(fd, &byte, 1), byte != startByte);

        if(read(fd, buf, numBytes) != numBytes){
            cerr << "read failed"  << endl;
            continue;
        }

        if(read(fd, &byte, 1), byte != endByte){
            ROS_INFO("End byte not found");
            continue;
        }

        //cout
        //    << "[" << counter[0] << "]"
        //    << "[" << counter[1] << "]"
        //    << "[" << counter[2] << "]"
        //    << endl;

        c.data = buf[0];
        pubA.publish(c);
        c.data = buf[1];
        pubB.publish(c);
        c.data = buf[2];
        pubC.publish(c);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
