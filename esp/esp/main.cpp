#include <Arduino.h>


extern "C" {
    int app_main(void){
        Serial.begin(115200);
        while(1){
            Serial.println("hello");
        }

    }
}
