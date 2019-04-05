#include <ros/ros.h>
#include "control.h"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <trinity/GetRobotPose.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "control");
        
    ros::NodeHandle controlNode;
    ros::Publisher velPub = controlNode.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::ServiceClient mapClient = controlNode.serviceClient<nav_msgs::GetMap>("getMap");
    ros::ServiceClient robotPoseClient = controlNode.serviceClient<trinity::GetRobotPose>("GetRobotPose");
    Control control(mapClient, velPub, robotPoseClient);
    ros::Subscriber waitForSignal = controlNode.subscribe("startSignal", 100, &Control::controlLoop, &control);
    ros::spin();
}


