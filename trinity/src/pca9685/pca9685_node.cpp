#include "pca9685.h"

#include <string>

#include <ros/ros.h>
#include <std_msgs/UInt32.h>

using std::string;

const char *node_name = "pca9685_node";
const char *i2c_interface = "/dev/i2c-1";

int main(int argc, char **argv){
    ros::init(argc, argv, node_name); 
    ros::NodeHandle n;

    string topic_name;
    n.getParam("pwm", topic_name);

    PCA9685 pca(i2c_interface);

    typedef std_msgs::UInt32::ConstPtr input_type;
    typedef boost::function<void (const input_type&)> callback_func;
    //TODO: fix type here
    callback_func callback = [&pca](const input_type& val){ pca.setDutyCycle(0, 0.f); };

    //can we remove sub?
    ros::Subscriber sub = n.subscribe(topic_name, 100, callback);

    ros::spin();

    return 0;
}
