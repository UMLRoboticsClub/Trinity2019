//this is like mazemapper comboed with robot from last year
#include <nav_msgs/OccupancyGrid.h>

enum ROBOT_OPS {
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
	Control()
	void controlLoop();
	geometry_msgs::Pose findNextTarget(nav_msgs::OccupancyGrid msg);
	void takeAction(RobotOp robotAction);
	Point computeDistanceField();
	RobotOp determineRobotOp(int);
	void extinguishCandle();
	bool unknownLargeEnough(Point center);

private:
	gameState gs;
	map<int, vector<geometry_msgs::Pose>> targetPoints;
	vector<vector<int>> distanceField;
	nav_msgs::OccupancyGrid occGrid;
	MoveBaseClient ac;
	ros::ServiceClient client;
}
