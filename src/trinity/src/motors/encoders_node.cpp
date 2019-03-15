#include "encoder.h"
#include "gpio.h"

#include "ros/ros.h"
#include "std_msgs/Int64.h"

const char *node_name = "encoders";
const char *topic_nameA = "enc_a";
const char *topic_nameB = "enc_b";
const char *topic_nameC = "enc_c";
const unsigned pub_hz = 20; 

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    if(!gpioConnect()){ return 1; }

    //init and set motor power to 0
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
        pubA.publish(c);

        c.data = encoderB.count;
        pubB.publish(c);

        c.data = encoderC.count;
        pubC.publish(c);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
