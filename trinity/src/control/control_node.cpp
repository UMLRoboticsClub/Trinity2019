#include <ros/ros.h>
#include "control.h"
#include <nav_msgs/GetMap.h>


int main(int argc, char* argv[]){
    ros::init(argc, argv, "control");
        
    ros::NodeHandle controlNode;
    ros::ServiceClient mapClient = controlNode.serviceClient<nav_msgs::GetMap>("getMap");
    Control control(mapClient);
    ros::Subscriber waitForSignal = controlNode.subscribe("startSignal", 100, &Control::controlLoop, &control);
    ros::spin();
}


