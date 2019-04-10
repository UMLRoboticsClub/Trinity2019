#include "TCS34725.h"
#include <ros/ros.h>
#include <trinity_pi/GetInRoom.h>
#include <iostream>

#define WHITE_THRESHOLD 150
#define COUNT_THRESHOLD 20

using std::string;

bool inRoom = false;

bool getInRoom(trinity_pi::GetInRoom::Request&, trinity_pi::GetInRoom::Response& res){
    res.inRoom.data = inRoom;
    return true;
}

int getColorReading(TCS34725 &colorSensor);


int main(int argc, char* argv[]){
    ros::init(argc, argv, "color");
    ros::NodeHandle colorNode("");
	string interface = "/dev/i2c-1";

    //ros::ServiceClient robotPoseClient = controlNode.serviceClient<trinity::GetRobotPose>("GetRobotPose");
	colorNode.getParam("~i2c_device", interface);
	
    ros::ServiceServer server = colorNode.advertiseService("GetInRoom", getInRoom);
    TCS34725 colorSensor(interface.c_str());
    ros::Rate loop_rate(20);
    int reading;
    int whiteCount = 0;
    bool offStartPlatform = false;
    
    while(ros::ok()){
        reading = getColorReading(colorSensor);
        ROS_INFO("%d", reading);
        if(reading < WHITE_THRESHOLD){
            if (whiteCount > COUNT_THRESHOLD){
                //we crossed a white region of significant size
                if(offStartPlatform){
                    inRoom = !inRoom;
                }
            }
            offStartPlatform = true;
            whiteCount = 0;
        }
        else{//currently over white
            whiteCount++;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

int getColorReading(TCS34725 &colorSensor){
    float r, g, b;
    colorSensor.getRGB(&r, &g, &b);
    return (r + g + b)/3.f;
}

