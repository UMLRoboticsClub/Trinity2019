//this is like mazemapper comboed with robot from last year
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include "point.h"
#include <move_base_msgs/MoveBaseAction.h>
#include "gamestate.h"
#include <map>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <trinity_pi/GetRobotPose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

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
    Control(ros::NodeHandle* nodehandle);
	void controlLoop(const nav_msgs::OccupancyGrid::ConstPtr&);
	void startFunc(const std_msgs::Bool::ConstPtr&);
    geometry_msgs::Pose findNextTarget(RobotOp& op);
	void takeAction(RobotOp robotAction);
    Point computeDistanceField();
	RobotOp determineRobotOp(int);
	void extinguishCandle(geometry_msgs::Pose candlePose);
	bool unknownLargeEnough(Point center);
    int accessOccGrid(int x, int y);
    vector<Point> findOpenNeighbors(const Point &currentPos);
    vector<Point> findUnknownNeighbors(const Point &currentPos);
    vector<Point> findNeighbors(const Point &currentPos);
    bool isDiag(int x, int y);
    Point poseToPoint(geometry_msgs::Pose pose);
    geometry_msgs::Pose pointToPose(Point point);
    geometry_msgs::Pose getRobotPose();
    double irSense();
    vector<double> parseIrReadings(vector<double>);
	bool start;
private:
    ros::Publisher cmd_vel_pub;
    ros::Publisher goal_pub;
	ros::Publisher point_pub;
	ros::Subscriber map_sub;
	ros::Subscriber wait_for_signal;
	ros::NodeHandle nh;
    GameState gs;
    std::map<int, vector<geometry_msgs::Pose>> targetPoints;
	vector<vector<int>> distanceField;
	nav_msgs::OccupancyGrid occGrid;
	MoveBaseClient* ac;
    ros::ServiceClient robotPoseClient;
    ros::ServiceClient irClient;
    ros::ServiceClient solenoidClient;
	ros::ServiceClient inRoomClient;
	void initializeSubscribers();
	void initializePublishers();
	void initializeServices();
};
