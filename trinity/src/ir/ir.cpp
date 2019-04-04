#include "ir.h"
#include "gpio.h"

#include "gpio.h"
#include <pigpiod_if2.h>

IRSensor::IRSensor(){
    gpioConnect();
    set_mode(0, IR_SENSOR, PI_INPUT);
}

IRSensor::~IRSensor(){
    gpioDisconnect();
}

bool flameVisible(){
    return !gpio_read(0, IR_SENSOR);
}
