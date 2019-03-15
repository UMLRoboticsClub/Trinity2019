#include "encoder.h"
#include "gpio.h"

#include "ros/ros.h"
#include "std_msgs/Int64.h"

const char *node_name = "encoders";
const char *topic_name = "encoders_pub";
const unsigned pub_hz = 20; 

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;

    if(!gpioConnect()){ return 1; }

    //init and set motor power to 0
    Encoder encoderA(MOTORA_ENCA, MOTORA_ENCB);
    Encoder encoderB(MOTORB_ENCA, MOTORB_ENCB);
    Encoder encoderC(MOTORC_ENCA, MOTORC_ENCB);

    ros::Publisher pub = n.advertise<std_msgs::Int64>(topic_name, 1);
    ros::Rate loop_rate(pub_hz);

    std_msgs::Int64 c;
    while(ros::ok()){
        c.data = encoderA.count;
        pub.publish(c);

        c.data = encoderB.count;
        pub.publish(c);

        c.data = encoderC.count;
        pub.publish(c);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
