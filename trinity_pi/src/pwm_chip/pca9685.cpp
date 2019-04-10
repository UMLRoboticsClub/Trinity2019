#include "pca9685.h"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <chrono>
#include <thread>

#define PCA9685_SUBADR1  0x2
#define PCA9685_SUBADR2  0x3
#define PCA9685_SUBADR3  0x4

#define PCA9685_MODE1    0x0
#define PCA9685_PRESCALE 0xFE
#define PCA9685_RESET    0x6

#define LED0_ON_L        0x6
#define LED0_ON_H        0x7
#define LED0_OFF_L       0x8
#define LED0_OFF_H       0x9

#define ALLLED_ON_L      0xFA
#define ALLLED_ON_H      0xFB
#define ALLLED_OFF_L     0xFC
#define ALLLED_OFF_H     0xFD

//#define ENABLE_DEBUG_OUTPUT

using std::cout;
using std::cerr;
using std::endl;

const int default_freq = 500;

inline void sleep(int millis){
	std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

//addr: The 7-bit I2C address to locate this chip, default is 0x40
PCA9685::PCA9685(const char *device, uint8_t addr){
    if((fd = open(device, O_RDWR)) < 0){
        cerr << "Unable to open I2C interface" << endl;
        exit(1);
    }
    if(ioctl(fd, I2C_SLAVE, addr)  < 0){
        cerr << "Unable to connect to PCA9685" << endl;
        exit(1);
    }

    //reset();

	//set a default frequency
    setPWMFreq(default_freq);
}

PCA9685::~PCA9685(){
    setAllPWM(0, PWM_LOW);
    close(fd);
}

//Sends a reset command to the PCA9685 chip over I2C
void PCA9685::reset(){
    uint8_t val = PCA9685_RESET;
    if(write(fd, &val, 1) != 1){ cerr << "pca reset: write failed" << endl; }
    sleep(10);
    setAllPWM(0, 4096);
}

void PCA9685::setDutyCycle(uint8_t pin, float percent){
    if(percent < 0 || percent > 1){ cerr << "bad input for duty cycle" << endl; }
    setPin(pin, (uint16_t)(PWM_MAX * percent));
}

//Sets the PWM frequency for the entire chip, up to ~1.6 KHz
//freq: Floating point frequency that we will attempt to match
void PCA9685::setPWMFreq(float freq){
#ifdef ENABLE_DEBUG_OUTPUT
    cout << "Attempting to set freq " << freq << endl;
#endif

    freq *= 0.9;  // Correct for overshoot in the frequency setting
    float prescaleval = 25000000;
    prescaleval /= PWM_MAX;
    prescaleval /= freq;
    prescaleval -= 1;

#ifdef ENABLE_DEBUG_OUTPUT
    cout << "Estimated pre-scale: " << prescaleval << endl;
#endif

    uint8_t prescale = prescaleval + 0.5f;
#ifdef ENABLE_DEBUG_OUTPUT
    cout << "Final pre-scale: " << prescale << endl;
#endif

    //uint8_t oldmode = read8(PCA9685_MODE1);
    //uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
    //write8(PCA9685_MODE1, newmode);          // go to sleep
    //write8(PCA9685_PRESCALE, prescale);      // set the prescaler
    //write8(PCA9685_MODE1, oldmode);
    //sleep(5);
    //write8(PCA9685_MODE1, oldmode | 0xa0);   //  This sets the MODE1 register to turn on auto increment.

    // Get settings and calc bytes for the different states.
    int settings = read8(PCA9685_MODE1) & 0x7F;   // Set restart bit to 0
    int sleep_m   = settings | 0x10;                                  // Set sleep bit to 1
    int wake    = settings & 0xEF;                                  // Set sleep bit to 0
    int restart = wake | 0x80;

    // Go to sleep, set prescale and wake up again.
    write8(PCA9685_MODE1, sleep_m);
    write8(PCA9685_PRESCALE, prescale);
    write8(PCA9685_MODE1, wake);

    // Now wait a millisecond until oscillator finished stabilizing and restart PWM.
    sleep(1);
    write8(PCA9685_MODE1, restart);

#ifdef ENABLE_DEBUG_OUTPUT
            cout << "Mode now 0x" << std::hex << read8(PCA9685_MODE1) << endl;
#endif
            }

            //Sets the PWM output of one of the PCA9685 pins
            //pin: One of the PWM output pins, from 0 to 15
            //on: At what point in the 4096-part cycle to turn the PWM output ON
            //off: At what point in the 4096-part cycle to turn the PWM output OFF
            void PCA9685::setPWM(uint8_t pin, uint16_t on, uint16_t off){
#ifdef ENABLE_DEBUG_OUTPUT
            cout << "Setting PWM " << pin << ": " << on << "->" << off << endl;
#endif

            write8(LED0_ON_L  + 4*pin, on);
            write8(LED0_ON_H  + 4*pin, on >> 8);
            write8(LED0_OFF_L + 4*pin, off);
            write8(LED0_OFF_H + 4*pin, off >> 8);
            }

void PCA9685::setAllPWM(uint16_t on, uint16_t off){
#ifdef ENABLE_DEBUG_OUTPUT
    cout << "Setting all PWM: " << on << "->" << off << endl;
#endif

    write8(ALLLED_ON_L,  on);
    write8(ALLLED_ON_H,  on >> 8);
    write8(ALLLED_OFF_L, off);
    write8(ALLLED_OFF_H, off >> 8);
}

void PCA9685::setPin(uint8_t pin, uint16_t val){
    //clamp value between 0 and 4095 inclusive
    if (val >= 4095) {
        // Special value for signal fully on.
        setPWM(pin, 4096, 0);
    } else if (val == 0) {
        // Special value for signal fully off.
        setPWM(pin, 0, 4096);
    } else {
        setPWM(pin, 0, val);
    }
}

void PCA9685::setAllPins(uint16_t val){
    //clamp value between 0 and 4095 inclusive
    if(val > 4095){ val = 4095; }

    if (val == 4095) {
        // Special value for signal fully on.
        setAllPWM(4096, 0);
    } else if (val == 0) {
        // Special value for signal fully off.
        setAllPWM(0, 4096);
    } else {
        setAllPWM(0, val);
    }
}

uint8_t PCA9685::read8(uint8_t addr){
    if(write(fd, &addr, 1) != 1){
        cerr << "read: pca write failed" << endl;
        return 0;
    }
    if(read(fd, &addr, 1)  != 1){
        cerr << "read: pca read failed" << endl;
        return 0;
    }

    return addr;
}

void PCA9685::write8(uint8_t addr, uint8_t val) {
    uint8_t packet[2] = { addr, val };
    if(write(fd, packet, 2) != 2){
        cerr << "pca9685 write failed(1)" << endl;
    }
}
