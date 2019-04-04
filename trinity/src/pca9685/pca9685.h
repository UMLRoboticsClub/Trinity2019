#pragma once

#include <cstdint>

const uint16_t PWM_MAX  = 4096;
const uint16_t PWM_HIGH = 4096;
const uint16_t PWM_LOW  = 0;

class PCA9685 {
    public:
        PCA9685(const char *device, uint8_t addr = 0x40);
        ~PCA9685();
        void reset(void);

        void setPWMFreq(float freq);

        //set duty cycle to normalized(!) float
        void setDutyCycle(uint8_t pin, float percent);

        void setAllPins(uint16_t val);
        void setPin(uint8_t pin, uint16_t val);

        void setPWM(uint8_t pin, uint16_t on, uint16_t off);
        void setAllPWM(uint16_t on, uint16_t off);

    private:
        int fd = 0;

        inline uint8_t read8(uint8_t addr);
        inline void write8(uint8_t addr, uint8_t val);
};
