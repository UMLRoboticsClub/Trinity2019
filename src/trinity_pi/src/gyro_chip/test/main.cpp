#include "../fxas21002c.h"

#include <iostream>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;

const char *interface = "/dev/i2c-1";

inline void sleep(int millis){
    std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

int main(){

    FXAS21002C gyro(interface);

    while(true){
        gyro.update();
        cout
            << "[" << gyro.data.x << "]"
            << "[" << gyro.data.y << "]"
            << "[" << gyro.data.x << "]"
            << endl;
        sleep(200);
    }

    return 0;
}
