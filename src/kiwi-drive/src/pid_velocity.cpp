#include "ros/ros.h"
#include <std_msgs/Float32.h>

const char *node_name = "pid_velocity";
const char *topic_motorA_src = "mtr_a_vel";
const char *topic_motorB_src = "mtr_b_vel";
const char *topic_motorC_src = "mtr_c_vel";
const char *topic_encA_src = "enc_a";
const char *topic_encB_src = "enc_b";
const char *topic_encC_src = "enc_c";
const char *topic_motorA_dst = "mtr_a";
const char *topic_motorB_dst = "mtr_b";
const char *topic_motorC_dst = "mtr_c";
const int bufferSize = 1000;

long enc_a_count = 0;
long enc_b_count = 0;
long enc_c_count = 0;

//callbacks to update a,b,c counts

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    //ros::Subscriber vel_sub;
    //ros::Publisher mtrA_pub, mtrB_pub, mtrC_pub;


    return 0;
}
