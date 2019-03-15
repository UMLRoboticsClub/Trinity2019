#pragma once

class Motor {
    public:
        Motor(unsigned pinA, unsigned pinB);
        ~Motor();
        //sets motor power [-255,255]
        void set(int power);
        //sets motor power to 0
        void stop();

    private:
        unsigned pinA, pinB;
};
