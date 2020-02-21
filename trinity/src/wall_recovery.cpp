#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <base_local_planner/costmap_model.h>
#include <nav_core/recovery_behavior.h>
#include <tf/transform_listener.h>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(wall_recovery::WallRecovery, nav_core::RecoveryBehavior)

namespace wall_recovery
{
WallRecovery::WallRecovery() : local_costmap_(NULL), initialized_(false), world_model_(NULL)
{
}

void WallRecovery::initialize(std::string name, tf::TransformListener *,
                              costmap_2d::Costmap2DROS *, costmap_2d::Costmap2DROS *local_costmap)
{
  if (!initialized_)
  {
    local_costmap_ = local_costmap;

    // get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle blp_nh("~/TrajectoryPlannerROS");

    // we'll simulate every degree by default
    private_nh.param("sim_granularity", sim_granularity_, 0.017);
    private_nh.param("frequency", frequency_, 20.0);

    blp_nh.param("acc_lim_th", acc_lim_th_, 3.2);
    blp_nh.param("max_rotational_vel", max_rotational_vel_, 1.0);
    blp_nh.param("min_in_place_rotational_vel", min_rotational_vel_, 0.4);
    blp_nh.param("yaw_goal_tolerance", tolerance_, 0.10);

    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else
  {
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

WallRecovery::~WallRecovery()
{
  delete world_model_;
}

// TODO 
// find closest wall
// drive away from it :) 
// create .h file w all the functions and local variables
// local variable for costmap
// local variable for robot position

void WallRecovery::runBehavior()
{
  if (!initialized_)
  {
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }

  if (local_costmap_ == NULL)
  {
    ROS_ERROR("The costmap passed to the WallRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Rotate recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  tf::Stamped<tf::Pose> global_pose;
  local_costmap_->getRobotPose(global_pose);

  double current_angle = tf::getYaw(global_pose.getRotation());
  double start_angle = current_angle;
}

Point WallRecovery::computeDistanceField(tf::Stamped<tf::Pose> robotPos)
{
  //does a breadth first search of the maze beginning at current robot position
  //until locating an unknown region of sufficient size
  //populates the distance field as it goes
  //KEY POINT make sure distance field is always reset properly between loops
  //also switch from vector to something circular

  vector<Point> boundary;
  boundary.push_back(robotPos.data.position);
  vector<Point> neighbors;
  distanceField[robotPos.data.position.x][robotPos.data.position.y] = 0;
  Point currentCell;
  int currentDistance;

  ROS_INFO("Computing distance field");
  while (ros::ok() && !boundary.empty())
  {
    //ROS_INFO("Inside while loop");
    currentCell = boundary.front();
    //ROS_INFO("Current cell: (%d, %d)", currentCell.x, currentCell.y);
    //ROS_INFO("Cell value: %d", accessOccGrid(currentCell.x, currentCell.y));
    neighbors = findOpenNeighbors(currentCell);
    currentDistance = distanceField[currentCell.x][currentCell.y];
    for (Point neighbor : neighbors)
    {
      if (distanceField[neighbor.x][neighbor.y] == -1)
      { //neighbor not already indexed by function
        // if point is unknown, check to see if local area is made up of unknowns as well
        if (accessOccGrid(neighbor.x, neighbor.y) == 99)
        {
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

}; // namespace wall_recovery