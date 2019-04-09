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
    ros::Publisher velPub = controlNode.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	ros::Publisher goalPub = controlNode.advertise<geometry_msgs::PoseStamped>("goal", 1);
	ros::Publisher pointPub = controlNode.advertise<geometry_msgs::PointStamped>("robot_point", 1);
    ros::ServiceClient robotPoseClient = controlNode.serviceClient<trinity_pi::GetRobotPose>("GetRobotPose");
    ros::ServiceClient irClient = controlNode.serviceClient<std_srvs::Trigger>("GetFlame");
    ros::ServiceClient solenoidClient = controlNode.serviceClient<std_srvs::Empty>("Extinguish");
    Control control(pointPub, velPub, goalPub, robotPoseClient, irClient, solenoidClient);
	ros::Subscriber sub = controlNode.subscribe("/move_base/global_costmap/costmap", 1, &Control::controlLoop, &control);
    ros::Subscriber waitForSignal = controlNode.subscribe("startSignal", 100, &Control::startFunc, &control);
	control.start = true;
    ros::spin();
}


