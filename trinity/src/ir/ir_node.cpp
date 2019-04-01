#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "gpio.h"
#include <pigpiod_if2.h>

const char *node_name = "ir";
const char *topic_name = "ir_pub";
const unsigned pub_hz = 10; 

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    if(!gpioConnect()){ return 1; }

    ros::Publisher pub = n.advertise<std_msgs::Bool>(topic_name, 1);
    ros::Rate loop_rate(pub_hz);

    std_msgs::Bool b;
    while(ros::ok()){
        b.data = !gpio_read(0, IR_SENSOR);
        pub.publish(b);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
