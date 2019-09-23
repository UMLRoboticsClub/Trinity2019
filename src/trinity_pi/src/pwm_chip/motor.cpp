#include "motor.h"

#include <iostream>

using std::cerr;
using std::endl;

Motor::Motor(PCA9685 &pca, unsigned pinA, unsigned pinB):
    pca(pca), pinA(pinA), pinB(pinB) {}

Motor::~Motor(){
    stop();
}

void Motor::stop(){
    pca.setPin(pinA, PWM_LOW);
    pca.setPin(pinB, PWM_LOW);
}

void Motor::set(float power){
    //printf("got power: %.3f", power);
    //if(power == lastVal){ return; };
    printf("setting power: %.3f\n", power);
    if(power < 0){
        pca.setDutyCycle(pinA, 0);
        pca.setDutyCycle(pinB, -power);
    } else {
        pca.setDutyCycle(pinB, 0);
        pca.setDutyCycle(pinA, power);
    }
    lastVal = power;
}
