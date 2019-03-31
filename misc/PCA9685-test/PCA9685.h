#pragma once

#include <cstdint>

#define PCA9685_SUBADR1  0x2
#define PCA9685_SUBADR2  0x3
#define PCA9685_SUBADR3  0x4

#define PCA9685_MODE1    0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L        0x6
#define LED0_ON_H        0x7
#define LED0_OFF_L       0x8
#define LED0_OFF_H       0x9

#define ALLLED_ON_L      0xFA
#define ALLLED_ON_H      0xFB
#define ALLLED_OFF_L     0xFC
#define ALLLED_OFF_H     0xFD

class PCA9685 {
    public:
        PCA9685(const char *device, uint8_t addr = 0x40);
        ~PCA9685();
        void reset(void);

        void setPWMFreq(float freq);

        void setAllPins(uint16_t val);
        void setPin(uint8_t num, uint16_t val);

        void setPWM(uint8_t num, uint16_t on, uint16_t off);
        void setAllPWM(uint16_t on, uint16_t off);

    private:
        int fd = 0;
		const char *device;
        uint8_t addr;

        uint8_t read8(uint8_t addr);
        void write8(uint8_t addr, uint8_t d);
};
