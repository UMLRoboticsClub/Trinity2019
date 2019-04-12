#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cmath>

const char *node_name = "joystick_drive";
const char *topic_name = "cmd_vel";

static int linear    = 1;
static int angular   = 2;
static double lscale = 1;
static double ascale = 1;

geometry_msgs::Twist twist;
int main(int argc, char **argv){
    ros::init(argc, argv, node_name);
    ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    
	ros::NodeHandle nh;
    nh.param("axis_linear",   linear,  linear);
    nh.param("axis_angular",  angular, angular);
    nh.param("scale_angular", ascale,  ascale);
    nh.param("scale_linear",  lscale,  lscale);

    boost::function<void (const sensor_msgs::Joy::ConstPtr &joy)> callback = 
        [&vel_pub](const sensor_msgs::Joy::ConstPtr &joy){
			float x = lscale*joy->axes[linear];
			float y = lscale*joy->axes[0];
			float norm = sqrt(pow(x, 2) + pow(y,2));
            twist.angular.z = ascale*joy->axes[angular];
			if(norm != 0){
            	twist.linear.x = lscale*joy->axes[linear] / norm * 0.75;
				twist.linear.y = lscale*joy->axes[0] / norm * 0.75;
			} else {
				twist.linear.x = 0;
				twist.linear.y = 0;
			}
        };

    vel_pub = nh.advertise<geometry_msgs::Twist>(topic_name, 1);
    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, callback);
	ros::Rate loop_rate(10);
	while(ros::ok()){
		vel_pub.publish(twist);
		ros::spinOnce();
		loop_rate.sleep();	
	}
}
