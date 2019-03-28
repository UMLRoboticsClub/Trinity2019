#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/OccupancyGrid.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Control::Control(): gs(), targetPoints(), ac("move_base", true){
	//initialize the distance field
	client = n.serviceClient<nav_msgs::GetMap>("GetMap");
	distanceField = vector<vector<int>>(GRID_SIZE_CELLS);
    for(unsigned i = 0; i < distanceField.size(); ++i){
        distanceField[i] = vector<int>(GRID_SIZE_CELLS);
        for(unsigned j = 0; j < distanceField[i].size(); ++j){
            distanceField[i][j] = -1;
        }
    }
}

void Control::controlLoop(){
	ros::init();
	move_base_msgs::MoveBaseGoal goal;
	RobotOp robotAction;
	nav_msgs::GetMap occGridService;
	while(!gs.done){
		//ask for occupancy grid service call to get current map.  store in occGrid.
		occGrid = client.call(occGridService);
		//determine target and type based on occGrid and gameState
		Point target = findNextTarget(occGrid , robotAction);
		//translate occGrid coords into moveBase coords
		geometry_msgs::Pose origin = occGrid.MapMetaData.Origin;
		//move moveBase
		move_base_msgs::MoveBaseGoal goal;

		goal.target_pose.header.frame_id = "/map";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = target.x - origin.position.x;
		goal.target_pose.pose.position.y = target.y - origin.position.y;

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
    vector<int> primaryTargets = state.getTargetType();
    for (const int type : primaryTargets) {
        // if we have a destination in mind
        if (targetPoints[type].size() > 0) {
            //update robotOps
            robotAction = determineRobotOp(type);
			Point closestTarget = targetPoints[type][0];
            removeTargetPoint(type, targetIndex);
			targetPose.position.x = closestTarget.x;
			targetPose.position.y = closestTarget.y;
            return targetPose;
        }
    }

    //no important points already found, go to distance field and find an unknown
    targetLocation = computeDistanceField();
	geometry_msgs::Pose origin = occGrid.MapMetaData.Origin;
	targetPose.position.x = targetLocation.x - origin.position.x;
	targetPose.position.y = targetLocation.y - origin.position.y;
	return targetPose;
}

Point Control::computeDistanceField() {
	//does a breadth first search of the maze beginning at current robot position
	//until locating an unknown region of sufficient size
	//populates the distance field as it goes
	//KEY POINT make sure distance field is always reset properly between loops
	//also switch from vector to something circular

	//get the robot position
	robotPos = [GET_ROBOT_POS_SERVICE_CALL];

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
                if (occGrid.data[neighbor.x][neighbor.y] == -1 && unknownLargeEnough(neighbor)){
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
			gs.done == true;
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
			//rotate in circle, ir and camera should actually be constantly gathering data.
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.pose.rotation.w = 2*3.1415926535;
			ac.sendGoal(goal);
			while(!ac.getState().isDone()){
				//query ir sensor, determine if there is a candle
				//not sure how to do that, but whatever
				//we can just look for local maxima I suppose.  Again, test consistency.
				if(irSense()){
					geometry_msgs::Pose robotPose = [GET_ROBOT_POSE];
					double angle = robotPose.rotation.z;
					//multiple candles can be in same room, this is hard now.
					//experiment with ir sensor to determine how to do this.
					//if analog, easy, record local maxima
					//test consistency of flame sensor
					//if super consistent, just detect regions of "on" and point to their centers.
					//or we could literally use flame sensor to align properly once we figure stuff out.
				}
				//use angles w/ ir data to determine direction candles are in
				//face candle, call takeRobotAction(extinguish)
				//after extinguishing candle, repeat this call and check if there is still a candle

				//alternatively, store robot pose in targets.
			}
			break;
		case OP_EXIT_ROOM:
			break; //why does this exist?
		default:
			break;
	}
}

Control::robotOps Control::determineRobotOp(int type){
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
	//wait for however long
	//deactivate solenoid
}

bool Control::unknownLargeEnough(Point center){
	int areaSize  = 2; // (2 results in an inclusive 3*3 grid)5
	for(int i = center.x - (areaSize - 1); i < center.x + areaSize; i++){
		for(int j = center.y - (areaSize - 1); j < center.y + areaSize; j++){
			if(occGrid.getValue(i, j) != -1) return false; // something other than unknown has been found, so return false
		}
	}
	return true;
}
