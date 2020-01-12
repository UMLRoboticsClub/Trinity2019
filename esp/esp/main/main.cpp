#include "ros.h"
#include "std_msgs/String.h"

#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void rosserial_setup(){
      // Initialize ROS
      nh.initNode();
        nh.advertise(chatter);
}

void rosserial_publish(){
      str_msg.data = hello;
        // Send the message
        chatter.publish(&str_msg);
          nh.spinOnce();
}

extern "C" {
    void app_main(void){
        rosserial_setup();

        while(1){
            nh.spinOnce();
            vTaskDelay(100);
        }
    }
}

