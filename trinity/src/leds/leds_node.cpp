#include "gpio.h"

#include <ros/ros.h>
#include <std_msgs/Bool.h>

const char *node_name = "leds";

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    //exit if we can't connect to GPIO system
    if(!gpioConnect()){ return 1; }
    
    //set up solenoid pin
    set_mode(0, LED_FIRE,    PI_OUTPUT);
    set_mode(0, LED_VIDEO,   PI_OUTPUT);
    set_mode(0, LED_SND_ACT, PI_OUTPUT);

    typedef std_msgs::Bool::ConstPtr input_type;
    typedef boost::function<void (const input_type&)> callback_func;

    callback_func callback1 = [](const input_type& vel){ gpio_write(0, LED_FIRE,    vel->data); }; 
    callback_func callback2 = [](const input_type& vel){ gpio_write(0, LED_VIDEO,   vel->data); }; 
    callback_func callback3 = [](const input_type& vel){ gpio_write(0, LED_SND_ACT, vel->data); }; 

    //subscribe to solenoid messages
    ros::Subscriber sub1 = n.subscribe("led_fire",    10, callback1);
    ros::Subscriber sub2 = n.subscribe("led_video",   10, callback2);
    ros::Subscriber sub3 = n.subscribe("led_snd_act", 10, callback3);

    //process callbacks
    ros::spin();

    gpio_write(0, LED_FIRE,    PI_LOW);
    gpio_write(0, LED_VIDEO,   PI_LOW);
    gpio_write(0, LED_SND_ACT, PI_LOW);

    gpioDisconnect();

    return 0;
}
