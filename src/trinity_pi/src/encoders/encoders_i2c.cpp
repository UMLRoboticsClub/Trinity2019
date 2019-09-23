#include <errno.h>
#include <cstring>
#include <stdio.h>
#include <cstdint>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

using std::string;

char buf[12];

union Encoders{
    uint8_t b[12];
    int32_t enc[3];
} enc;
 
int i2c_init(const char* filename, int addr) {
    int file;
 
    if ((file = open(filename,O_RDWR)) < 0) {
        printf("Failed to open the bus.");
        exit(1);
    }
 
    if (ioctl(file,I2C_SLAVE,addr) < 0) {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }
    return file;
}
 
 
//////////
// Set pointer address
//////////
void i2c_set_pointer(int address, int file) {   
    char buf[10];
    buf[0]=address;
    //printf("set pointer\n"); 
    int ret = write(file, buf, 1);
    if (ret != 1) {
        fprintf(stderr, "Error setting pointer\n");
    } else {
        //printf("w_0x%0*x\n", 2, buf[0]);
    }
}
 
//////////
// Read n bytes
//////////
char * i2c_read(int add1, int nbytes,int file) {
    //int n;
    //char* fail = "fail";
 
    i2c_set_pointer(add1,file);
    uint8_t startByte = 170;
    uint8_t endByte = 169;
    bool p = false;
    uint8_t byte;
    bool done = false;
    //while(ros::ok()){
        for(int i = 0; i < 12; i++){
            read(file, &byte, 1);
            /*if(byte == startByte){
                printf("%dF-", byte);
                p = false;
            } else if(byte == endByte) {
                printf("%dT-", byte);
                p = true;
            } else if(p) {
                printf("%dX-", byte);
            }*/
            printf("%d-", byte);
        }
        printf("\n");
    //    i2c_set_pointer(add1, file);
    //}
    return 0;
    while(ros::ok() && !done){
        //ROS_INFO("WTF");
        read(file, &byte, 1);
        int count = 0;
        while(ros::ok() && byte != startByte)
        {
            count++;
            printf("byte is: %d\n", byte);
            //ROS_INFO("%d", byte == startByte);
            read(file, &byte, 1); 
            if(count > 13){
                return 0;
            }
        }
    
        printf("{%d-", byte);
        if(read(file, buf, nbytes) != nbytes){
            ROS_ERROR("read failed");
            continue;
        } 
        for(int i = 0; i < nbytes; i++){
            printf("%d-", buf[i]);
        }
        read(file, &byte, 1);
        printf("%d}\n", byte);
        done = true;
        /**if(read(file, &byte, 1), byte != endByte){
            ROS_INFO("end byte not found!");
            continue;
        } else {
            printf("\nend byte: %d\n", byte); 
            done = true;
        }**/
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
    return buf;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "encoders");
    ros::NodeHandle n("~");
    string topic_nameA, topic_nameB, topic_nameC, i2c_device;
    int slave_address, file, pub_hz;
    char* buffer;

    topic_nameA = n.param<std::string>("encoder1", "enc1");
    topic_nameB = n.param<std::string>("encoder2", "enc2");
    topic_nameC = n.param<std::string>("encoder3", "enc3");
    i2c_device = n.param<std::string>("i2c_device", "/dev/i2c-1");
    slave_address = n.param("slave_address", 0x03);
    pub_hz = n.param("pub_hz", 20);

    ROS_INFO("encoders node / encoder1: %s, encoder2: %s, encoder3: %s, i2c_device: %s, slave_address: 0x%x, pub_hz: %d", topic_nameA.c_str(), topic_nameB.c_str(), topic_nameC.c_str(), i2c_device.c_str(), slave_address, pub_hz);
	file = i2c_init(i2c_device.c_str(), slave_address);
    ROS_INFO("Opened i2c bus on address %x", slave_address);

    ros::Publisher pubA = n.advertise<std_msgs::Int32>(topic_nameA, 1);
    ros::Publisher pubB = n.advertise<std_msgs::Int32>(topic_nameB, 1);
    ros::Publisher pubC = n.advertise<std_msgs::Int32>(topic_nameC, 1);
    ros::Rate loop_rate(pub_hz);
    std_msgs::Int32 c;
    
    //i2c_set_pointer(0x07, file);

    ROS_INFO("Publishing encoder values");
    while(ros::ok()){
        buffer = i2c_read(0x05, 12, file); //read at address 0x05
        /*if(buffer == 0){
            close(file);
            file = i2c_init(i2c_device.c_str(), slave_address);
            loop_rate.sleep();
            continue;
        }
        for(int i = 0; i < 12; i++){
            enc.b[i] = buffer[i];
        }
        c.data = enc.enc[0];
        pubA.publish(c);
        c.data = enc.enc[1];
        pubB.publish(c);
        c.data = enc.enc[2];
        pubC.publish(c);
*/
        //printf("%d, %d, %d\n", enc.enc[0], enc.enc[1], enc.enc[2]);
        ros::spinOnce();
        loop_rate.sleep();
    }
	close(file);
}
