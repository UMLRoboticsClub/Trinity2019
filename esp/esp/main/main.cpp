#include "ros.h"
#include "std_msgs/String.h"

#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher ack("ack", &str_msg);

void messageCb( const std_msgs::String& msg){
    ack.publish(&msg);
}

ros::Subscriber<std_msgs::String> echo("echo", &messageCb);

void rosserial_setup(){
    // Initialize ROS
    nh.initNode();
    nh.subscribe(echo);
    nh.advertise(ack);
}

void rosserial_spinonce(){
    nh.spinOnce();
}

extern "C" {
    void app_main(void){
        rosserial_setup();

        while(1){
            rosserial_spinonce();
            vTaskDelay(100);
        }
    }
}

