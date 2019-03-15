#include "encoder.h"

#include "gpio.h"

//holy shit this is ugly
Encoder::Encoder(unsigned pinA, unsigned pinB): pinA(pinA), pinB(pinB){
    callbackA = callback_ex(0,pinA, RISING_EDGE,
            [](int, unsigned, unsigned, uint32_t, void *data){
                Encoder &e = *static_cast<Encoder*>(data);
                if(!gpio_read(0, e.pinB)){
                    ++e.count; 
                }
            },
            this);
    callbackB = callback_ex(0,pinB, RISING_EDGE,
            [](int, unsigned, unsigned, uint32_t, void *data){
                Encoder &e = *static_cast<Encoder*>(data);
                if(!gpio_read(0, e.pinA)){
                    --e.count; 
                }
            },
            this);
}

Encoder::~Encoder(){
    callback_cancel(callbackA);
    callback_cancel(callbackB);
}
