#include "ir.h"
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

IRSensor flameSensor;

bool getIr(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res){
    res.success = flameSensor.flameVisible();
    return true;
}

int main(int argc, char* argv[]){
    flameSensor = IRSensor();
    ros::init(argc, argv, "ir");
    ros::NodeHandle irNode;
    ros::ServiceServer server = irNode.advertiseService("GetFlame", getIr);
}
