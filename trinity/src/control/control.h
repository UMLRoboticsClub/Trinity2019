//this is like mazemapper comboed with robot from last year
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include "point.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "gamestate.h"
#include <map>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

typedef enum ROBOT_OPS {
	OP_NOTHING,
	OP_CRADLE_LEFT,
	OP_CRADLE_RIGHT,
	OP_CRADLE_FRONT,
	OP_SAFE_ZONE,
	OP_EXTINGUISH,
	OP_SCANROOM,
	OP_EXIT_ROOM,
	OP_HALLWAY,
	OP_HALLWAY_SIMPLE,
	OP_STOP
} RobotOp;

class Control{
public:
    Control(ros::ServiceClient&, ros::Publisher, ros::ServiceClient&);
	void controlLoop(const std_msgs::Bool::ConstPtr& sig);
    geometry_msgs::Pose findNextTarget(RobotOp& op);
	void takeAction(RobotOp robotAction);
    Point computeDistanceField();
	RobotOp determineRobotOp(int);
	void extinguishCandle();
	bool unknownLargeEnough(Point center);
    vector<Point> findOpenNeighbors(const Point &currentPos);
    int accessOccGrid(int x, int y);
    bool isDiag(int x, int y);
    Point poseToPoint(geometry_msgs::Pose pose);
    geometry_msgs::Pose pointToPose(Point point);
    geometry_msgs::Pose getRobotPose();
    double irSense();
    vector<double> parseIrReadings(vector<double>);
private:
    ros::Publisher cmd_vel_pub;
    GameState gs;
    std::map<int, vector<geometry_msgs::Pose>> targetPoints;
	vector<vector<int>> distanceField;
	nav_msgs::OccupancyGrid occGrid;
	MoveBaseClient ac;
	ros::ServiceClient mapClient;
    ros::ServiceClient robotPoseClient;
};
