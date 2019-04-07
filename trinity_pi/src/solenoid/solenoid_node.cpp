#include "gpio.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
const char *node_name = "solenoid";

bool extinguish(std_srvs::Empty::Request&, std_srvs::Empty::Response&){
    gpio_write(0, SOLENOID, 1);
    ros::Duration(1).sleep();
    gpio_write(0, SOLENOID, 0);
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    //exit if we can't connect to GPIO system
    if(!gpioConnect()){ return 1; }
    
    //set up solenoid pin
    set_mode(0, SOLENOID, PI_OUTPUT);

    ros::ServiceServer server = n.advertiseService("Extinguish", extinguish);

    //process callbacks

    gpioDisconnect();

    return 0;
}
