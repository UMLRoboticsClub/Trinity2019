#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "control.h"
#include <nav_msgs/GetMap.h>
#include "gpio.h"
/*
TODO
implement robotPositionGet function call
test this shit
*/





Control::Control(ros::ServiceClient& mapClient, ros::Publisher velPub, ros::ServiceClient& robotPoseClient): gs(), targetPoints(), ac("move_base", true){
	//initialize the distance field
	this->mapClient = mapClient;
    cmd_vel_pub = velPub;
    this->robotPoseClient = robotPoseClient;
	distanceField = vector<vector<int>>(GRID_SIZE_CELLS);
    for(unsigned i = 0; i < distanceField.size(); ++i){
        distanceField[i] = vector<int>(GRID_SIZE_CELLS);
        for(unsigned j = 0; j < distanceField[i].size(); ++j){
            distanceField[i][j] = -1;
        }
    }
}

void Control::controlLoop(const std_msgs::Bool::ConstPtr& sig){
	move_base_msgs::MoveBaseGoal goal;
	RobotOp robotAction;
	nav_msgs::GetMap occGridService;
	while(!gs.done){
		//ask for occupancy grid service call to get current map.  store in occGrid.
		mapClient.call(occGridService);
        occGrid = occGridService.response.map;
		//determine target and type based on occGrid and gameState
        geometry_msgs::Pose target = findNextTarget(robotAction);
		//translate occGrid coords into moveBase coords
		//move moveBase
		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose = target;

		ac.sendGoal(goal);
		ac.waitForResult();

		//perform necessary action at targetLoc.
		//if we entered a room, update robotAction accordingly
		//update robot action in case we entered a room
		takeAction(robotAction);
	}
}

//modify this to return a pose instead of a point
//correct for origin inside this function.
geometry_msgs::Pose Control::findNextTarget(RobotOp& robotAction){
	//last year's code modified for accessing occGrid.  Nice and simple
	robotAction = OP_NOTHING;
    for(unsigned i = 0; i < distanceField.size(); ++i){
        for(unsigned j = 0; j < distanceField[i].size(); j ++){
            distanceField[i][j] = -1;
        }
    }
	geometry_msgs::Pose targetPose;
    //check if we have already found an important point where we need to go
    vector<int> primaryTargets = gs.getTargetType();
    for (const int type : primaryTargets) {
        // if we have a destination in mind
        if (targetPoints[type].size() > 0) {
            //update robotOps
            robotAction = determineRobotOp(type);
            geometry_msgs::Pose closestTarget = targetPoints[type][0];
            int targetIndex = 0;
			targetPoints[type].erase(targetPoints[type].begin()+targetIndex);
            return closestTarget;
        }
    }

    //no important points already found, go to distance field and find an unknown
    return pointToPose(computeDistanceField());
}

Point Control::computeDistanceField() {
	//does a breadth first search of the maze beginning at current robot position
	//until locating an unknown region of sufficient size
	//populates the distance field as it goes
	//KEY POINT make sure distance field is always reset properly between loops
	//also switch from vector to something circular

	//get the robot position
    Point robotPos = poseToPoint(getRobotPose());

    vector<Point> boundary;
    boundary.push_back(robotPos);
    vector<Point> neighbors;
    distanceField[robotPos.x][robotPos.y] = 0;
    Point currentCell;
    int currentDistance;

    while (!boundary.empty()) {
        currentCell = boundary.front();
        neighbors = findOpenNeighbors(currentCell);
        currentDistance = distanceField[currentCell.x][currentCell.y];
        for (Point neighbor : neighbors) {
            if(distanceField[neighbor.x][neighbor.y] == -1){//neighbor not already indexed by function
                // if point is unknown, check to see if local area is made up of unknowns as well
                if (accessOccGrid(neighbor.x, neighbor.y) == -1 && unknownLargeEnough(neighbor)){
                    // this point actually does represent an unkown region`
                    distanceField[neighbor.x][neighbor.y] = currentDistance + 1;
                    return neighbor;
                }
                distanceField[neighbor.x][neighbor.y] = currentDistance + 1;
                boundary.push_back(neighbor);
            }
        }
        boundary.erase(boundary.begin());
    }
    return Point(-1, -1);
}

void Control::takeAction(RobotOp robotAction){
	//based on given action type, determine what action needs to be taken.
	//call the necessary helper function
	//update gameState accordingly
	switch(robotAction){
		case OP_STOP:
			gs.done = true;
			break;
		case OP_HALLWAY_SIMPLE:
			gs.secondArena = false;
			break;
		case OP_HALLWAY:
			//bunch of stuff is gonna go here, but for now...
			gs.secondArena = true;
			break;
		case OP_EXTINGUISH:
			extinguishCandle();
			break;
		case OP_SCANROOM:
            {
            //rotate in circle, ir and camera should actually be constantly gathering data.
			vector<double> irReadings = vector<double>();
            geometry_msgs::Twist rotCommand;
            rotCommand.angular.z = 1;
            geometry_msgs::Pose robotPose = getRobotPose();
            geometry_msgs::Pose newRobotPose;
            double delta = 0;
            //while we haven't rotated 2*PI rads
            while(delta < 2*3.1415926535){
                irReadings.push_back(irSense());
                cmd_vel_pub.publish(rotCommand);
                newRobotPose = getRobotPose();
                delta += newRobotPose.orientation.z - robotPose.orientation.z;
                robotPose = newRobotPose;
            }
            //candles is vector of angle indices relative to robot orientation
            vector<double> candles = parseIrReadings(irReadings);
			for(double candle : candles){
                geometry_msgs::Pose candlePose;
                candlePose.position = robotPose.position;
                candlePose.orientation.z = robotPose.orientation.z + candle;
                targetPoints[FLAME].push_back(candlePose);
            }
            break;
            }
		case OP_EXIT_ROOM:
			break; //why does this exist?
		default:
			break;
	}
}

RobotOp Control::determineRobotOp(int type){
    switch(type){
        case START_ZONE:
            return OP_STOP;
        case HALLWAY:
            return gs.secondArena ? OP_HALLWAY_SIMPLE : OP_HALLWAY;
        case FLAME:
        case CANDLE:
            return OP_EXTINGUISH;
        case DOOR:
            return OP_SCANROOM;
        case EXPLORED_DOOR:
            return OP_EXIT_ROOM;
        case SAFE_ZONE:
            return OP_SAFE_ZONE;
        case FRONT_SIDE_CRADLE:
            return OP_CRADLE_FRONT;
        case LEFT_SIDE_CRADLE:
            return OP_CRADLE_LEFT;
        case RIGHT_SIDE_CRADLE:
            return OP_CRADLE_RIGHT;
        default:
            return OP_NOTHING;
    }
}

//robot already facing correct direction,
void Control::extinguishCandle(){
	//activate solenoid
	gpio_write(0, SOLENOID, 1);
    //wait for however long
    ros::Duration(1).sleep();
	//deactivate solenoid
    gpio_write(0, SOLENOID, 0);
}

bool Control::unknownLargeEnough(Point center){
	int areaSize  = 2; // (2 results in an inclusive 3*3 grid)5
	for(int i = center.x - (areaSize - 1); i < center.x + areaSize; i++){
		for(int j = center.y - (areaSize - 1); j < center.y + areaSize; j++){
			if(accessOccGrid(i, j) != -1) return false; // something other than unknown has been found, so return false
		}
	}
	return true;
}

vector<Point> Control::findOpenNeighbors(const Point &currentPos) {
	vector<Point> openNeighbors;
	for (int x_offset = -1; x_offset < 2; ++x_offset) {
		for (int y_offset = -1; y_offset < 2; ++y_offset) {
			if (!isDiag(x_offset, y_offset) &&
					accessOccGrid(currentPos.x + x_offset, currentPos.y + y_offset) <= CLEAR_THRESHOLD) {
				openNeighbors.push_back(Point(currentPos.x + x_offset, currentPos.y + y_offset));
			}
		}
	}
	return openNeighbors;
}


int Control::accessOccGrid(int x, int y){
    return occGrid.data[x*occGrid.info.width + y];
}

bool Control::isDiag(int x, int y){
    return (abs(x) + abs(y) != 1);
}

Point Control::poseToPoint(geometry_msgs::Pose pose){
    //convert from real world pose to occGrid point.
    int realX = int(pose.position.x/occGrid.info.resolution);
    int realY = int(pose.position.y/occGrid.info.resolution);

    return Point(realX-occGrid.info.origin.position.x, realY-occGrid.info.origin.position.y);
}

geometry_msgs::Pose Control::pointToPose(Point point){
    int originizedX = point.x + occGrid.info.origin.position.x;
    int originizedY = point.y + occGrid.info.origin.position.y;

    geometry_msgs::Pose pose;
    pose.position.x = originizedX*occGrid.info.resolution;
    pose.position.y = originizedY*occGrid.info.resolution;

    return pose;
}

geometry_msgs::Pose Control::getRobotPose(){
    trinity::GetRobotPose srv;
   robotPoseClient.call(srv);
   if (srv.response.pose.header.frame_id != occGrid.header.frame_id){
       ROS_INFO("Oh shit frames are different need to fix");
   }
   return srv.response.pose.pose;
}

double Control::irSense(){
 return !gpio_read(0, IR_SENSOR); 
}

vector<double> Control::parseIrReadings(vector<double> readings){
    //find regions of HIGH readings, center of regions indicates a flame
    //return vector of angles in radians for candle direction.
    //just add a special case if it wraps around
    double readingThreshold = 0.5;
    int angleThreshold = 10;
    vector<double> candles = vector<double>();
    int endIndex = readings.size();

    int sigStart;
    if (readings[0] > readingThreshold){
        //iterate in reverse to find start of region
        int index = 0;
        while(readings[--index+readings.size()] > readingThreshold);
        sigStart = index+1;
        endIndex = index;
    }

    for(int i = 1; i < endIndex; i ++){
        if(readings[i] > readingThreshold && readings[i-1] < readingThreshold)
            sigStart = i;
        else if(readings[i] < readingThreshold && readings[i-1] > readingThreshold){
            //end of candle reading range.
            if((i - sigStart)*2*3.1415926535/readings.size() > angleThreshold){
                double midIndex = (i-1 - sigStart)/2;
                double angle = midIndex/readings.size() * 2 * 3.1415926535;
                candles.push_back(angle);
            }
        }
    }
    return candles;
}
