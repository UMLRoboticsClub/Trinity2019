#include "encoder.h"
#include "gpio.h"

#include <ros/ros.h>
#include <std_msgs/Int64.h>

using std::string;

const char *node_name = "encoders";
const unsigned pub_hz = 20; 

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");
	string topic_nameA, topic_nameB, topic_nameC;

	n.getParam("encoder1", topic_nameA);
	n.getParam("encoder2", topic_nameB);
	n.getParam("encoder3", topic_nameC);

    if(!gpioConnect()){ return 1; }

    Encoder encoderA(MOTORA_ENCA, MOTORA_ENCB);
    Encoder encoderB(MOTORB_ENCA, MOTORB_ENCB);
    Encoder encoderC(MOTORC_ENCA, MOTORC_ENCB);

    ros::Publisher pubA = n.advertise<std_msgs::Int64>(topic_nameA, 1000);
    ros::Publisher pubB = n.advertise<std_msgs::Int64>(topic_nameB, 1000);
    ros::Publisher pubC = n.advertise<std_msgs::Int64>(topic_nameC, 1000);
    ros::Rate loop_rate(pub_hz);

    std_msgs::Int64 c;
    while(ros::ok()){
        c.data = encoderA.count;
        c.data = encoderB.count;
        c.data = encoderC.count;

        pubA.publish(c);
        pubB.publish(c);
        pubC.publish(c);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
