#include "pca9685.h"
#include "motor.h"
#include "pins.h"

#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

using std::string;

const char *node_name = "motors";
const char *i2c_interface = "/dev/i2c-1";
const int msgBufSize = 1;

static PCA9685 pca(i2c_interface);

static Motor motorA(pca, MOTORA_PINA, MOTORA_PINB);
static Motor motorB(pca, MOTORB_PINA, MOTORB_PINB);
static Motor motorC(pca, MOTORC_PINA, MOTORC_PINB);

static ros::Subscriber s_mtrA, s_mtrB, s_mtrC, s_ledF, s_ledV, s_ledS;

void motorA_callback(std_msgs::Float32::ConstPtr cmd){
    ROS_INFO("got command 2: %.3f", cmd->data);
}

void init_motors(ros::NodeHandle &n){
    string topic_motorA, topic_motorB, topic_motorC;
    n.param<std::string>("motor1", topic_motorA, "/motor1_cmd");
    n.param<std::string>("motor2", topic_motorB, "/motor2_cmd");
    n.param<std::string>("motor3", topic_motorC, "/motor3_cmd");
    ROS_INFO("%s, %s, %s", topic_motorA.c_str(), topic_motorB.c_str(), topic_motorC.c_str());
    typedef std_msgs::Float32::ConstPtr mtr_input_type;
    typedef boost::function<void (const mtr_input_type&)> callback_func;

    callback_func mtrMsgA = [](const mtr_input_type& vel){ motorA.set(-vel->data); };
    callback_func mtrMsgB = [](const mtr_input_type& vel){ motorB.set(-vel->data); };
    callback_func mtrMsgC = [](const mtr_input_type& vel){ motorC.set(-vel->data); };

    //subscribe to motor messages
    s_mtrA = n.subscribe(topic_motorA, msgBufSize, mtrMsgA);
    s_mtrB = n.subscribe(topic_motorB, msgBufSize, mtrMsgB);
    s_mtrC = n.subscribe(topic_motorC, msgBufSize, mtrMsgC);
}

void init_leds(ros::NodeHandle &n){
    typedef std_msgs::Bool::ConstPtr input_type;
    typedef boost::function<void (const input_type&)> callback_func;

    callback_func callback1 = [](const input_type& vel){
        pca.setPin(LED_FIRE, vel->data ? PWM_HIGH : PWM_LOW);
    }; 
    callback_func callback2 = [](const input_type& vel){
        pca.setPin(LED_VIDEO, vel->data ? PWM_HIGH : PWM_LOW);
    }; 
    callback_func callback3 = [](const input_type& vel){
        pca.setPin(LED_SND_ACT, vel->data ? PWM_HIGH : PWM_LOW);
    }; 

    //subscribe to solenoid messages
    s_ledF = n.subscribe("led_fire",    10, callback1);
    s_ledV = n.subscribe("led_video",   10, callback2);
    s_ledS = n.subscribe("led_snd_act", 10, callback3);
}

int main(int argc, char **argv){
    ros::init(argc, argv, node_name); 
    ros::NodeHandle n("~");

    init_motors(n);
    init_leds(n);

    //process callbacks
    ros::spin();

    return 0;
}
