#include "gpio.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
const char *node_name = "solenoid";

bool extinguish(std_srvs::Empty::Request&, std_srvs::Empty::Response&){
    ROS_INFO("triggering solenoid");
    gpio_write(0, SOLENOID, 1);
    ros::Duration(0.1).sleep();
    gpio_write(0, SOLENOID, 0);
    ROS_INFO("stopping solenoid");
    return true;
}

int main(int argc, char **argv){
    printf("1");
    ros::init(argc, argv, node_name);
    printf("2");
    ros::NodeHandle n;
    printf("3");

    //exit if we can't connect to GPIO system
    if(!gpioConnect()){ return 1; }
    printf("4");
    //set up solenoid pin
    set_mode(0, SOLENOID, PI_OUTPUT);
    printf("5");
    ROS_INFO("excuse me what the fuck");
    ROS_INFO("hello senor we are doin a start");
    ros::ServiceServer server = n.advertiseService("Extinguish", extinguish);
    ROS_INFO("pip-pip de doodly doo/n");
    //process callbacks
    ros::spin();
    gpioDisconnect();

    return 0;
}
