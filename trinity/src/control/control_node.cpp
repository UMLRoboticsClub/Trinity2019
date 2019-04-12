#include <ros/ros.h>
#include "control.h"
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <trinity_pi/GetRobotPose.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "control");
    ros::NodeHandle controlNode;
    Control control(&controlNode);
    ros::spin();
}


