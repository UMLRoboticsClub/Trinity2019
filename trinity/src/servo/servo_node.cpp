#include "gpio.h"

#include <string>
#include <cstdint>

#include <ros/ros.h>
#include <std_msgs/Float32.h>

using std::string;

const char *node_name = "servo";
const uint32_t max_duty_cycle = 1000000;

inline uint32_t angleToDuty(float angle){
    //TODO: check/test this
    return max_duty_cycle * angle;
}

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    if(!gpioConnect()){ return 1; }

    string topicName;
    n.getParam("servo", topicName);

    typedef std_msgs::Float32::ConstPtr mtr_input_type;
    typedef boost::function<void (const mtr_input_type&)> callback_func;

    callback_func callback = [](const mtr_input_type& val){
        hardware_PWM(0, SERVO, 50, angleToDuty(val->data));
    };

    ros::Subscriber sub = n.subscribe(topicName, 100, callback);

    ros::spin();

    hardware_PWM(0, SERVO, 50, 0);
    gpioDisconnect();

    return 0;
}
