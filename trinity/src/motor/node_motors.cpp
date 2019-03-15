#include "motor.h"
#include "gpio.h"

#include "ros/ros.h"
#include "std_msgs/String.h"

const char *node_name = "motors";

int main(int argc, char **argv){

    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    //exit if we can't connect to GPIO system
    if(!gpioConnect()){ exit(1); }

    //init and set motor power to 0
    Motor motorA(MOTORA_PINA, MOTORA_PINB);
    Motor motorB(MOTORB_PINA, MOTORB_PINB);
    Motor motorC(MOTORC_PINA, MOTORC_PINB);

    auto mtrMsgA = [&motorA](const std_msgs::Uint32 vel){ motorA.set(vel); };
    auto mtrMsgB = [&motorB](const std_msgs::Uint32 vel){ motorB.set(vel); };
    auto mtrMsgC = [&motorC](const std_msgs::Uint32 vel){ motorC.set(vel); };

    //subscribe to motor messages
    ros::Subscriber sub_a = n.subscribe("mtr_a", 1000, mtrMsgA);
    ros::Subscriber sub_b = n.subscribe("mtr_b", 1000, mtrMsgB);
    ros::Subscriber sub_c = n.subscribe("mtr_c", 1000, mtrMsgC);

    //process callbacks
    ros::spin();

    return 0;
}

void chatterCallback()
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
