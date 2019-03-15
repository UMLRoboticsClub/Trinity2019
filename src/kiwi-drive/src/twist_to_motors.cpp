//convert twist to motor power

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

const char *node_name = "twist_to_motors";
const char *topic_vel_src_param_name = "vel_src";
const char *topic_vel_src = "cmd_vel";
const char *topic_motorA = "mtr_a_vel";
const char *topic_motorB = "mtr_b_vel";
const char *topic_motorC = "mtr_c_vel";
const int topicBuffer = 1000;

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle n;
    nh.param(topic_vel_src_param_name, topic_vel_src, topic_vel_src);

    ros::Subscriber vel_sub;
    ros::Publisher mtrA_pub, mtrB_pub, mtrC_pub;

    boost::function<void (const geometry_msgs::Twist::ConstPtr &vel)> callback = 
        [&](const sensor_msgs::Twist::ConstPtr &vel){
            UInt32 a, b, c;
            //some math here
            //https://stackoverflow.com/questions/3748037/how-to-control-a-kiwi-drive-robot

            //void omni_drive(double x, double y, double theta) {
            //    float vc = -0.5 * vx - sqrt(3)/2 * vy+theta;
            //    float vb = -0.5 * vx + sqrt(3)/2 * vy+theta;
            //    float va = vx+theta;
            //}

            mtrA_pub.publish(a);
            mtrB_pub.publish(b);
            mtrC_pub.publish(c);
        };

    vel_sub = nh.subscribe<geometry_msgs::Twist>(topic_vel_src, topicBufer, callback);
    mtrA_pub = nh.advertise<std_msgs::UInt32>(topic_motorA, topicBuffer);
    mtrB_pub = nh.advertise<std_msgs::UInt32>(topic_motorB, topicBuffer);
    mtrC_pub = nh.advertise<std_msgs::UInt32>(topic_motorC, topicBuffer);

    ros::spin();

    return 0;
}
