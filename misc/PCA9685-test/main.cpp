#include "PCA9685.h"

#include <signal.h>
#include <iostream>
#include <thread>
#include <chrono>

using std::cout;
using std::endl;

bool running = true;
void signal_handler(int){ 
    cout << "\naborting..." << endl;
    running = false;
    //exit(0);
}

inline void sleep(int millis){
	std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

#define PWM_MAX 4096
#define FULL_ON 4096
#define FULL_OFF 0

int main(void) {
    printf("PCA9685 LED example\n");
    signal(SIGINT, signal_handler);

    PCA9685 pca("/dev/i2c-1");
    pca.setPWMFreq(50);

    //unsigned delta = 32;
    //while (running) {
    //    for (int i = 0; i < PWM_MAX; i += delta) {
    //        pca.setPin(0, i);
    //        sleep(4);
    //    }

    //    for (int i = 0; i < PWM_MAX; i += delta) {
    //        pca.setPin(0, PWM_MAX - i);
    //        sleep(4);
    //    }
    //}

    pca.setPin(0, PWM_MAX/2);
    //pca.setPin(0, FULL_OFF);
    //pca.setPin(0, FULL_ON);
    while(running);

    return 0;
}

