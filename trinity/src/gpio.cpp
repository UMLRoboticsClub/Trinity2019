#include "gpio.h"

#include <iostream>
#include <pigpiod_if2.h>

using std::cerr;
using std::endl;

bool gpioConnect(){
    if(pigpio_start(0, 0) < 0){
        cerr << "Unable to connect to pigpiod" << endl;
        return false;
    }
    return true;
}

void gpioDisconnect(){
    pigpio_stop(0);
}
