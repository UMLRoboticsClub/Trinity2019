#include <iostream>
#include <pigpiod_if2.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

const char *nodeName = "statuslight";

const int lightpin = 2;
const int delayms = 400;

const int maxBrightness = 128;

int main(int argc, char* argv[]){
    ros::init(argc, argv, nodeName);
    if((pigpio_start(0, 0)) < 0){
        fprintf(stderr, "Error: Unable to connect to pigpiod\n");
        exit(1);
    }

    set_mode(0, lightpin, PI_OUTPUT);  
    set_PWM_dutycycle(0, lightpin, maxBrightness);

    int brightness = 0;
    char increm = 1;

    while(ros::ok()){
        set_PWM_dutycycle(0, lightpin, brightness);
        brightness += increm;
        if(brightness == (maxBrightness - 15)){
            increm = -1;
        } else if(brightness == 5){
            increm = 1;
        }
        time_sleep(20/1000.f);
    }
    gpio_write(0, lightpin, 0);
    pigpio_stop(0);
}
