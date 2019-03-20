#include "gpio.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>

const char *node_name = "solenoid";

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    //exit if we can't connect to GPIO system
    if(!gpioConnect()){ return 1; }
    
    //set up solenoid pin
    set_mode(0, SOLENOID, PI_OUTPUT);

    typedef std_msgs::Bool::ConstPtr input_type;
    typedef boost::function<void (const input_type&)> callback_func;

    callback_func callback = [](const input_type& vel){ gpio_write(0, SOLENOID, vel->data); }; 
    //subscribe to solenoid messages
    ros::Subscriber sub_a = n.subscribe("solenoid", 100, callback);

    //process callbacks
    ros::spin();

    gpioDisconnect();

    return 0;
}
