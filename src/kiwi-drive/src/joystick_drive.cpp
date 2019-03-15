#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

const char *node_name = "joystick_drive";
const char *topic_name = "joystick_drive/cmd_vel";

static int linear    = 1;
static int angular   = 2;
static double lscale = 1;
static double ascale = 1;

int main(int argc, char **argv){
    ros::init(argc, argv, node_name);

    ros::NodeHandle nh;
    nh.param("axis_linear",   linear,  linear);
    nh.param("axis_angular",  angular, angular);
    nh.param("scale_angular", ascale,  ascale);
    nh.param("scale_linear",  lscale,  lscale);

    boost::function<void (const sensor_msgs::Joy::ConstPtr &joy)> callback = 
        [&vel_pub](const sensor_msgs::Joy::ConstPtr &joy){
            geometry_msgs::Twist twist;
            twist.angular.z = ascale*joy->axes[angular];
            twist.linear.x = lscale*joy->axes[linear];
            vel_pub.publish(twist);
        };

    auto vel_pub = nh.advertise<geometry_msgs::Twist>(topic_name, 1);
    auto joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, callback);

    ros::spin();
}
