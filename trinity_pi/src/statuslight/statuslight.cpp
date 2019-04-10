#include <iostream>
#include <ros/ros.h>

#include "gpio.h"
#include "pins.h"

const char *nodeName = "statuslight";
const int delayms = 400;
const int maxBrightness = 128;

int main(int argc, char* argv[]){
    ros::init(argc, argv, nodeName);
    ros::NodeHandle n;
    if(!gpioConnect()){ return 1; }

    set_mode(0, LED_STATUS, PI_OUTPUT);  
    set_PWM_dutycycle(0, LED_STATUS, maxBrightness);

    int brightness = 0;
    char increm = 1;

    while(ros::ok()){
        set_PWM_dutycycle(0, LED_STATUS, brightness);
        brightness += increm;
        if(brightness == (maxBrightness - 15)){
            increm = -1;
        } else if(brightness == 5){
            increm = 1;
        }
        time_sleep(20/1000.f);
    }
    gpio_write(0, LED_STATUS, 0);

    gpioDisconnect();
}
