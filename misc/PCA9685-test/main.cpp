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

//int main(){
//    signal(SIGINT, signal_handler);
//
//    PCA9685 pca("/dev/i2c-1");
//
//    //pca.setDutyCycle(0, .1f); 
//
//    //pca.setPin(0, 4096/2);
//    //pca.setPin(0, 4096);
//    //pca.setPin(0, 0);
//
//    //pca.setAllPWM(0, 4096);
//    //pca.setAllPWM(0, 4096/2);
//    //while(running);
//    //pca.setAllPWM(0, 0);
//    //pca.setPin(0, 0);
//
//    //50hz -> 1/50 sec = 20ms
//
//    float angle = 0;
//    float delta = 0.01f;
//    bool up = true;
//
//    pca.setPWMFreq(50);
//    //pca.setPWM(0, 0, 4096/2);
//    while(running){
//        if(up){
//            angle += delta;
//            if(angle > 1){
//                angle = 1;
//                up = false;
//            }
//        } else {
//            angle -= delta;
//            if(angle < 0){
//                angle = 0;
//                up = true;
//            }
//        }
//
//        int on = angle * 4096;
//        int off = 4096 - on;
//
//        pca.setPWM(0, on, off);
//        //pca.setPin(0, on);
//        sleep(100);
//    }
//
//    //while(running){
//    //    sleep(500);
//    //    pca.setPin(0, 1/20.f * 4096);
//    //    sleep(500);
//    //    pca.setPin(0, 2/20.f * 4096);
//    //}
//
//    //pca.setPin(0, 1/20.f * 4096);
//
//
//    return 0;
//}


#define MAX_PWM 4096
#define FULL_ON 4096
#define FULL_OFF 0

int main(void) {
    printf("PCA9685 LED example\n");
    signal(SIGINT, signal_handler);

    PCA9685 pca("/dev/i2c-1");
    pca.setPWMFreq(50);

    //unsigned delta = 32;
    //while (running) {
    //    for (int i = 0; i < MAX_PWM; i += delta) {
    //        pca.setPin(0, i);
    //        sleep(4);
    //    }

    //    for (int i = 0; i < MAX_PWM; i += delta) {
    //        pca.setPin(0, MAX_PWM - i);
    //        sleep(4);
    //    }
    //}

    pca.setPin(0, MAX_PWM/2);
    //pca.setPin(0, FULL_OFF);
    //pca.setPin(0, FULL_ON);
    while(running);

    return 0;
}

