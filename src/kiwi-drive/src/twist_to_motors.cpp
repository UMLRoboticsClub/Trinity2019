//convert twist to motor power

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <ros/console.h>

const char *node_name = "twist_to_motors";
const char *topic_vel_src = "cmd_vel";
const char *topic_motorA = "mtr_a_vel";
const char *topic_motorB = "mtr_b_vel";
const char *topic_motorC = "mtr_c_vel";
const int bufferSize = 1000;

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    ROS_DEBUG("%s started", node_name);

    ros::Subscriber vel_sub;
    ros::Publisher mtrA_pub, mtrB_pub, mtrC_pub;

    std_msgs::Float32 a, b, c;
    float &va = a.data;
    float &vb = b.data;
    float &vc = c.data;

    boost::function<void (const geometry_msgs::Twist::ConstPtr &vel)> callback = 
        [&](const geometry_msgs::Twist::ConstPtr &vel){
            //for reference:
            //https://stackoverflow.com/questions/3748037/how-to-control-a-kiwi-drive-robot
            //Coordinate systems in ROS are always in 3D,
            //and are right-handed, with X forward, Y left, and Z up. 

            float vx = vel->linear.x;
            float vy = vel->linear.y;
            float theta = vel->angular.z;
            vc = -0.5f * vx - sqrt(3)/2 * vy+theta;
            vb = -0.5f * vx + sqrt(3)/2 * vy+theta;
            va = vx+theta;

            mtrA_pub.publish(a);
            mtrB_pub.publish(b);
            mtrC_pub.publish(c);

            ROS_DEBUG("publishing: (%d, %d, %d)", va, vb, vc);
    };

    vel_sub = nh.subscribe<geometry_msgs::Twist>(topic_vel_src, bufferSize, callback);
    mtrA_pub = nh.advertise<std_msgs::Float32>(topic_motorA, bufferSize);
    mtrB_pub = nh.advertise<std_msgs::Float32>(topic_motorB, bufferSize);
    mtrC_pub = nh.advertise<std_msgs::Float32>(topic_motorC, bufferSize);
    
    ros::spin();
    
    return 0;
}
