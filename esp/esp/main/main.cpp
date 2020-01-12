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

void rosserial_spinonce(){
    nh.spinOnce();
}

void delay_ms(size_t ms){
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

extern "C" {
    void app_main(void){
        rosserial_setup();

        while(1){
            rosserial_spinonce();
            delay_ms(1000);
        }
    }
}

