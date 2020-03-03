#include "control.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/GetMap.h>
#include <ros/ros.h>
#include <iostream>

using std::cin;
using std::cout;
using trinity::DoorArray;

Control::Control(ros::NodeHandle* nodeHandle) {
    nh = *nodeHandle;
    ROS_INFO("initializing");
    initializeSubscribers();
    initializePublishers();
    initializeServices();
    ROS_INFO("ros initialized");
    ac = new MoveBaseClient(nh, "move_base", true);
    while (!ac->waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    //initialize the distance field
    distanceField = vector<vector<int>>(GRID_SIZE_CELLS);
    for (unsigned i = 0; i < distanceField.size(); ++i) {
        distanceField[i] = vector<int>(GRID_SIZE_CELLS);
        for (unsigned j = 0; j < distanceField[i].size(); ++j) {
            distanceField[i][j] = -1;
        }
    }
    //ros::Duration(7).sleep();
    start = true;
    ROS_INFO("Done initializing");
}

void Control::initializeSubscribers() {
    map_sub = nh.subscribe("/move_base/global_costmap/costmap", 1, &Control::controlLoop, this);
    wait_for_signal = nh.subscribe("startSignal", 100, &Control::startFunc, this);
    get_doors = nh.subscribe("doors_array", 100, &Control::getDoors, this);
    ROS_INFO("done initializing subscribers");
}

void Control::initializePublishers() {
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal", 1);
    point_pub = nh.advertise<geometry_msgs::PointStamped>("robot_point", 1);
    ROS_INFO("done initializing publishers");
}

void Control::initializeServices() {
    robotPoseClient = nh.serviceClient<trinity_pi::GetRobotPose>("GetRobotPose");
    irClient = nh.serviceClient<std_srvs::Trigger>("GetFlame");
    solenoidClient = nh.serviceClient<std_srvs::Empty>("Extinguish");
    inRoomClient = nh.serviceClient<std_srvs::Empty>("GetInRoom");
    ROS_INFO("done initializing services");
}

void Control::startFunc(const std_msgs::Bool::ConstPtr&) {
    start = true;
}

void Control::getDoors(const DoorArray::ConstPtr& doors) {
    bool newDoor;
    ROS_INFO("got doors with size: %d", doors->doors.size());
    listener.waitForTransform(occGrid.header.frame_id, doors->header.frame_id, ros::Time::now(), ros::Duration(0.01));
    //ROS_INFO("finished waiting for transform...");
    for (geometry_msgs::Point doorP : doors->doors) {
        geometry_msgs::PointStamped ps;
        geometry_msgs::PointStamped pout;
        ps.header = doors->header;
        ps.point = doorP;

        listener.transformPoint(occGrid.header.frame_id, ros::Time(0), ps, ps.header.frame_id, pout);
        geometry_msgs::Pose pose;
        pose.position = pout.point;
        std_msgs::Header h;
        h.frame_id = occGrid.header.frame_id;
        h.stamp = ros::Time::now();
        Point door = poseToPoint(pose);
        //check if the new door is unique
        newDoor = true;
        for (Point explored : targetPoints[EXPLORED_DOOR]) {
            if (pointDist(explored, door) < NEW_DOOR_THRESH) {
                doorCount[explored]++;
                newDoor = false;
                break;
            }
        }
        if (!newDoor)
            continue;
        for (Point oldDoor : targetPoints[DOOR]) {
            if (pointDist(oldDoor, door) < NEW_DOOR_THRESH) {
                doorCount[oldDoor]++;
                newDoor = false;
                break;
            }
        }
        if (newDoor) {
            targetPoints[DOOR].push_back(door);
            doorCount[door] = 1;
        }
    }
}

// Create a marker for our marker array
visualization_msgs::Marker Control::CreateMarker(std_msgs::Header h, geometry_msgs::Pose p, float r, float g, float b, int id) {
double Control::pointDist(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}

int Control::findClosestDoorIndex() {
    Point robotPos = poseToPoint(getRobotPose());
    int closest = 0;
    for (int i = 1; i < targetPoints[DOOR].size(); i++) {
        if (pointDist(robotPos, targetPoints[DOOR][i]) < pointDist(robotPos, targetPoints[DOOR][closest]) && doorCount[targetPoints[DOOR][i]] > 10) {
            closest = i;
        }
    }
    return closest;
}

void Control::controlLoop(const nav_msgs::OccupancyGrid::ConstPtr& grid) {
    occGrid = *grid;
    move_base_msgs::MoveBaseGoal goal;
    RobotOp robotAction;
    ROS_INFO("In control loop");
    if (start && !gs.done) {
        //determine target and type based on occGrid and gameState
        geometry_msgs::Pose target = findNextTarget(robotAction);
        ROS_INFO("finished find next target: (%.2f, %.2f)", target.position.x, target.position.y);
        //translate occGrid coords into moveBase coords
        //move moveBase
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "/map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = target;
        goal.target_pose.pose.orientation.w = 1;
        geometry_msgs::PoseStamped psGoal;
        psGoal.pose = target;
        psGoal.header = goal.target_pose.header;
        goal_pub.publish(psGoal);
        ros::spinOnce();
        //ROS_INFO("goal pose: (%.2f, %.2f)", target.position.x, target.position.y);
        //ROS_INFO("Found goal, publishing...");
        char yn;
        cout << "Accept this goal? (y/n)" << endl;
        cin >> yn;
        if (yn == 'y') {
            ac->sendGoal(goal);
            ac->waitForResult();
            //TODO this is a temporary "solution".  Door code makes this obsolete
            //robotAction = OP_SCANROOM;
            //perform necessary action at targetLoc.
            //if we entered a room, update robotAction accordingly
            //`update robot action in case we entered a room
            takeAction(robotAction);
        }
    }
}

//modify this to return a pose instead of a point
//correct for origin inside this function.
geometry_msgs::Pose Control::findNextTarget(RobotOp& robotAction) {
    //last year's code modified for accessing occGrid.  Nice and simple
    robotAction = OP_NOTHING;
    //TODO uncomment this block of code to actually extinguish candles, find doors, etc
    geometry_msgs::Pose targetPose;
    //check if we have already found an important point where we need to go
    vector<int> primaryTargetTypes = gs.getTargetType();
    for (const int type : primaryTargetTypes) {
        ROS_INFO("target type: %d, size: %d", type, targetPoints[type].size());
        if (type == DOOR) {
            for (std::pair<Point, int> p : doorCount) {
                ROS_INFO("Point (%d, %d): %d", p.first.x, p.first.y, p.second);
            }
        }
        // if we have a destination in mind
        if (targetPoints[type].size() > 0) {
            //update robotOps
            robotAction = determineRobotOp(type);
            int targetIndex = type == DOOR ? findClosestDoorIndex() : 0;
            ROS_INFO("Is door: %d, index: %d", type == DOOR, targetIndex);
            Point closestTarget = targetPoints[type][targetIndex];
            targetPoints[type].erase(targetPoints[type].begin() + targetIndex);
            if (type == DOOR) {
                targetPoints[EXPLORED_DOOR].push_back(closestTarget);
                ROS_INFO("This door has been seen %d time(s)", doorCount[closestTarget]);
            }
            ROS_INFO("Closest target: (%d, %d)", closestTarget.x, closestTarget.y);
            return pointToPose(closestTarget);
        }
    }
    ROS_INFO("At end of find next target");
    //no important points already found, go to distance field and find an unknown
    return pointToPose(computeDistanceField());
}

Point Control::computeDistanceField() {
    //does a breadth first search of the maze beginning at current robot position
    //until locating an unknown region of sufficient size
    //populates the distance field as it goes
    //KEY POINT make sure distance field is always reset properly between loops
    //also switch from vector to a real queue

    //reset the distanceField
    for (unsigned i = 0; i < distanceField.size(); ++i) {
        for (unsigned j = 0; j < distanceField[i].size(); j++) {
            distanceField[i][j] = -1;
        }
    }

    //get the robot position
    Point robotPos = poseToPoint(getRobotPose());

    vector<Point> boundary;
    boundary.push_back(robotPos);
    vector<Point> neighbors;
    distanceField[robotPos.x][robotPos.y] = 0;
    Point currentCell;
    int currentDistance;

    ROS_INFO("Computing distance field");
    while (ros::ok() && !boundary.empty()) {
        //ROS_INFO("Inside while loop");
        currentCell = boundary.front();
        //ROS_INFO("Current cell: (%d, %d)", currentCell.x, currentCell.y);
        //ROS_INFO("Cell value: %d", accessOccGrid(currentCell.x, currentCell.y));
        neighbors = findOpenNeighbors(currentCell);
        currentDistance = distanceField[currentCell.x][currentCell.y];
        for (Point neighbor : neighbors) {
            if (distanceField[neighbor.x][neighbor.y] == -1) {  //neighbor not already indexed by function
                // if point is unknown, check to see if local area is made up of unknowns as well
                if (accessOccGrid(neighbor.x, neighbor.y) == -1 && unknownLargeEnough(neighbor)) {
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

void Control::takeAction(RobotOp robotAction) {
    //based on given action type, determine what action needs to be taken.
    //call the necessary helper function
    //update gameState accordingly
    switch (robotAction) {
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
            //TODO this doesn't exist unless we can actually detect candles
        case OP_EXTINGUISH:
            //extinguishCandle();
            break;
        case OP_SCANROOM: {
            //rotate in circle, ir and camera should actually be constantly gathering data.
            vector<double> irReadings = vector<double>();
            geometry_msgs::Twist rotCommand;
            rotCommand.angular.z = 1;
            geometry_msgs::Pose robotPose = getRobotPose();
            geometry_msgs::Pose newRobotPose;
            double delta = 0;
            double diff;
            //while we haven't rotated 2*PI rads
            //this should ahve a ROS rate
            ros::Rate rate(10);
            while (ros::ok() && delta < 2 * M_PI) {
                irReadings.push_back(irSense());
                cmd_vel_pub.publish(rotCommand);
                newRobotPose = getRobotPose();
                diff = (newRobotPose.orientation.z - robotPose.orientation.z);
                if (diff < 0)
                    diff += 2 * M_PI;
                delta += diff;
                robotPose = newRobotPose;
                rate.sleep();
            }
            //candles is vector of angle indices relative to robot orientation
            vector<double> candles = parseIrReadings(irReadings);
            for (double candle : candles) {
                geometry_msgs::Pose candlePose;
                candlePose.position = robotPose.position;
                candlePose.orientation.z = robotPose.orientation.z + candle;
                targetPoints[FLAME].push_back(poseToPoint(candlePose));
                extinguishCandle(candlePose);
            }
            break;
        }
        case OP_EXIT_ROOM:
            break;  //why does this exist?
        default:
            break;
    }
}

RobotOp Control::determineRobotOp(int type) {
    switch (type) {
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
void Control::extinguishCandle(geometry_msgs::Pose candlePose) {
    //move to the correct pose
    /*
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose = candlePose;
	ac->sendGoal(goal);
	ac->waitForResult();
    std_srvs::Empty srv;
	bool inRoom = inRoomClient.call(srv);
	//if(! inRoom){
	//if we are not in the room, approach the candle
	//technically should be able to be done with just cmd_vel to move forwards a little bit

	//}*/

    //TODO this should just be a goal that is the current position but rotated towards the candle

    double targetAngle = candlePose.orientation.z;
    double tolerance = 0.1;
    geometry_msgs::Pose robotPose = getRobotPose();
    geometry_msgs::Twist rotCommand;
    rotCommand.angular.z = 0;
    while (abs(robotPose.orientation.z - targetAngle) > tolerance) {
        double diff = targetAngle - robotPose.orientation.z;
        while (diff < -M_PI)
            diff += M_PI;
        while (diff > M_PI)
            diff -= M_PI;
        if (diff > 0)
            rotCommand.angular.z = 0.3;
        else
            rotCommand.angular.z = -0.3;
        cmd_vel_pub.publish(rotCommand);
        robotPose = getRobotPose();
    }
    std_srvs::Empty srv;
    solenoidClient.call(srv);

    //TODO this is only the case at level 1, shouldn't be part of this function
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = 0;
    goal.target_pose.pose.position.y = 0;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();
    ac->sendGoal(goal);
    ac->waitForResult();
    gs.done = true;
    return;
}

bool Control::unknownLargeEnough(Point center) {
    int distFromWall = 2;
    int minSize = 30;

    vector<Point> boundary;
    boundary.push_back(center);
    vector<Point> neighbors;
    distanceField[center.x][center.y] = 0;
    Point currentCell;
    int currentDistance;
    int areaCount = 1;

    while (ros::ok() && !boundary.empty()) {
        currentCell = boundary.front();
        neighbors = findNeighbors(currentCell);
        currentDistance = distanceField[currentCell.x][currentCell.y];
        for (Point neighbor : neighbors) {
            //ROS_INFO("occGrid: %d, distance: %d", accessOccGrid(neighbor.x, neighbor.y), currentDistance);
            if (accessOccGrid(neighbor.x, neighbor.y) >= 30 && currentDistance <= distFromWall) {
                //ROS_INFO("hit a wall, occGrid value of %d", accessOccGrid(neighbor.x, neighbor.y));
                //point is too close to a wall, abort mission
                return false;
            }
            if (distanceField[neighbor.x][neighbor.y] == -1) {  //neighbor not already indexed by function
                // if point is unknown, add to areaCount
                if (accessOccGrid(neighbor.x, neighbor.y) == -1) {
                    distanceField[neighbor.x][neighbor.y] = currentDistance + 1;
                    boundary.push_back(neighbor);
                    areaCount++;
                    if (areaCount > minSize) {
                        return true;
                    }
                }
            }
        }
        boundary.erase(boundary.begin());
    }
    return false;
}

vector<Point> Control::findOpenNeighbors(const Point& currentPos) {
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

vector<Point> Control::findUnknownNeighbors(const Point& currentPos) {
    vector<Point> openNeighbors;
    for (int x_offset = -1; x_offset < 2; ++x_offset) {
        for (int y_offset = -1; y_offset < 2; ++y_offset) {
            if (!isDiag(x_offset, y_offset) &&
                accessOccGrid(currentPos.x + x_offset, currentPos.y + y_offset) == -1) {
                openNeighbors.push_back(Point(currentPos.x + x_offset, currentPos.y + y_offset));
            }
        }
    }
    return openNeighbors;
}

vector<Point> Control::findNeighbors(const Point& currentPos) {
    vector<Point> openNeighbors;
    for (int x_offset = -1; x_offset < 2; ++x_offset) {
        for (int y_offset = -1; y_offset < 2; ++y_offset) {
            if (!isDiag(x_offset, y_offset)) {
                openNeighbors.push_back(Point(currentPos.x + x_offset, currentPos.y + y_offset));
            }
        }
    }
    return openNeighbors;
}

int Control::accessOccGrid(int x, int y) {
    if (x < 0 || x >= occGrid.info.width || y < 0 || y >= occGrid.info.height) {
        return 100;
    }
    return occGrid.data[y * occGrid.info.width + x];
}

bool Control::isDiag(int x, int y) {
    return (abs(x) + abs(y) != 1);
}

Point Control::poseToPoint(geometry_msgs::Pose pose) {
    //convert from real world pose to occGrid point.
    //ROS_INFO("pose - origin: %.3f", pose.position.x - occGrid.info.origin.position.x);
    //ROS_INFO("divided by res: %d", int((pose.position.x - occGrid.info.origin.position.x) / occGrid.info.resolution));
    //ROS_INFO("origin: (%.2f, %.2f), resolution: %.2f", occGrid.info.origin.position.x, occGrid.info.origin.position.y, occGrid.info.resolution);
    int realX = int((pose.position.x - occGrid.info.origin.position.x) / occGrid.info.resolution);
    int realY = int((pose.position.y - occGrid.info.origin.position.y) / occGrid.info.resolution);

    //ROS_INFO("(realX, realY): (%d, %d)", realX, realY);
    Point p = Point(realX, realY);
    //ROS_INFO("(p.x, p.y): (%d, %d)", p.x, p.y);
    return p;
}

geometry_msgs::Pose Control::pointToPose(Point point) {
    geometry_msgs::Pose pose;
    pose.position.x = point.x * occGrid.info.resolution + occGrid.info.origin.position.x;
    pose.position.y = point.y * occGrid.info.resolution + occGrid.info.origin.position.y;

    return pose;
}

geometry_msgs::Pose Control::getRobotPose() {
    trinity_pi::GetRobotPose srv;
    robotPoseClient.call(srv);
    ROS_INFO("pose frame: %s, map frame: %s", srv.response.pose.header.frame_id.c_str(), occGrid.header.frame_id.c_str());
    if (srv.response.pose.header.frame_id != occGrid.header.frame_id) {
        ROS_INFO("Oh shit frames are different need to fix");
    }
    return srv.response.pose.pose;
}

double Control::irSense() {
    std_srvs::Trigger srv;
    irClient.call(srv);
    return srv.response.success;
}

//TODO improve this - actually record robot angle per reading for one thing
vector<double> Control::parseIrReadings(vector<double> readings) {
    //find regions of HIGH readings, center of regions indicates a flame
    //return vector of angles in radians for candle direction.
    //just add a special case if it wraps around
    double readingThreshold = 0.5;
    int angleThreshold = 10;
    vector<double> candles = vector<double>();
    int endIndex = readings.size();

    int sigStart;
    if (readings[0] > readingThreshold) {
        //iterate in reverse to find start of region
        int index = 0;
        while (readings[--index + readings.size()] > readingThreshold)
            ;
        sigStart = index + 1;
        endIndex = index + readings.size();
    }

    for (int i = 1; i < endIndex; i++) {
        if (readings[i] > readingThreshold && readings[i - 1] < readingThreshold)
            sigStart = i;
        else if (readings[i] < readingThreshold && readings[i - 1] > readingThreshold) {
            //end of candle reading range.
            if (((i - sigStart + readings.size()) % readings.size()) * 2 * M_PI / readings.size() > angleThreshold) {
                double midIndex = ((i - 1 - sigStart + readings.size()) % readings.size() / 2);
                double angle = midIndex / readings.size() * 2 * M_PI;
                candles.push_back(angle);
            }
        }
    }
    return candles;
}
