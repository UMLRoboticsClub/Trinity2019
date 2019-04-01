#include "motor.h"
#include "gpio.h"

#include <ros/ros.h>
#include <std_msgs/UInt32.h>

using std::string;

const char *node_name = "motors";

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

	string topic_motorA, topic_motorB, topic_motorC;
	n.getParam("motor1", topic_motorA);
	n.getParam("motor2", topic_motorB);
	n.getParam("motor3", topic_motorC);

    //exit if we can't connect to GPIO system
    if(!gpioConnect()){ return 1; }

    //init motors
    Motor motorA(MOTORA_PINA, MOTORA_PINB);
    Motor motorB(MOTORB_PINA, MOTORB_PINB);
    Motor motorC(MOTORC_PINA, MOTORC_PINB);

    typedef std_msgs::UInt32::ConstPtr mtr_input_type;
    typedef boost::function<void (const mtr_input_type&)> callback_func;

    callback_func mtrMsgA = [&motorA](const mtr_input_type& vel){ motorA.set(vel->data); };
    callback_func mtrMsgB = [&motorB](const mtr_input_type& vel){ motorB.set(vel->data); };
    callback_func mtrMsgC = [&motorC](const mtr_input_type& vel){ motorC.set(vel->data); };

    //subscribe to motor messages
    ros::Subscriber sub_a = n.subscribe(topic_motorA, 100, mtrMsgA);
    ros::Subscriber sub_b = n.subscribe(topic_motorB, 100, mtrMsgB);
    ros::Subscriber sub_c = n.subscribe(topic_motorC, 100, mtrMsgC);

    //process callbacks
    ros::spin();

    gpioDisconnect();

    return 0;
}
