#include "TCS34725.h"
#include <ros/ros.h>
#include <trinity/GetRobotPose.h>
#include <trinity/GetInRoom.h>

#define WHITE_THRESHOLD 150
#define COUNT_THRESHOLD 20
const char *interface = "/dev/i2c-1";



bool inRoom = false;

bool getInRoom(trinity::GetInRoom::Request&, trinity::GetInRoom::Response& res){
    res.inRoom.data = inRoom;
    return true;
}

int getColorReading(TCS34725 colorSensor);


int main(int argc, char* argv[]){
    ros::init(argc, argv, "color");
    ros::NodeHandle colorNode;
    //ros::ServiceClient robotPoseClient = controlNode.serviceClient<trinity::GetRobotPose>("GetRobotPose");


    ros::ServiceServer server = colorNode.advertiseService("GetInRoom", getInRoom);

    TCS34725 colorSensor(interface);

    int reading;
    int whiteCount = 0;
    bool offStartPlatform = false;
    
    while(true){
        reading = getColorReading(colorSensor);
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
        
    }

    return 0;
}

int getColorReading(TCS34725 colorSensor){
    float r, g, b;
    colorSensor.getRGB(&r, &g, &b);
    return (r + g + b)/3.f;
}

