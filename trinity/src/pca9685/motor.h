#pragma once

#include "pca9685.h"

class Motor {
    public:
        Motor(PCA9685 &pca, unsigned pinA, unsigned pinB);
        //sets motor power [-1.f, 1.f]
        void set(float power);
        //sets motor power to 0
        void stop();

    private:
        //handle to PWM chip
        PCA9685 &pca;
        //pins for H-Bridge
        unsigned pinA, pinB;
};
