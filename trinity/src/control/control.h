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
#include <trinity/DoorArray.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

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
	void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& grid);
	void controlLoop();
	void startFunc(const std_msgs::Bool::ConstPtr&);
    geometry_msgs::Pose findNextTarget(RobotOp& op);
	void takeAction(RobotOp robotAction);
    Point computeDistanceField(Point, int);
	RobotOp determineRobotOp(int);
	void extinguishCandle(geometry_msgs::Pose candlePose);
	bool groupLargeEnough(Point center, int cellValue, int minSize);
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
	void getDoors(const trinity::DoorArray::ConstPtr& doors);
	double pointDist(Point a, Point b);
	int findClosestDoorIndex();
	visualization_msgs::Marker CreateMarker(std_msgs::Header h, geometry_msgs::Pose p, std::string text, float r, float g, float b, int id);
private:
    ros::Publisher cmd_vel_pub;
    ros::Publisher goal_pub;
	ros::Publisher point_pub;
	ros::Publisher seen_doors_pub;
	ros::Publisher target_points_pub;
	ros::Subscriber map_sub;
    ros::Subscriber get_doors;
	ros::Subscriber wait_for_signal;
	ros::NodeHandle nh;
	tf::TransformListener listener;
    GameState gs;
    std::map<int, vector<Point>> targetPoints;
	std::map<Point, int> doorCount;
	visualization_msgs::MarkerArray m_array;
	visualization_msgs::MarkerArray m_target_array;
	vector<vector<int>> distanceField;
	nav_msgs::OccupancyGrid occGrid;
	MoveBaseClient* ac;
    ros::ServiceClient robotPoseClient;
    ros::ServiceClient irClient;
    ros::ServiceClient solenoidClient;
	ros::ServiceClient inRoomClient;
	vector<Point> foundDoors;
		void initializeSubscribers();
	void initializePublishers();
	void initializeServices();
};
