#include <Arduino.h>

#include <driver/ledc.h>

int motor_pin = 13;

int main(){
    esp_err_t res;

    ledc_timer_config_t timer_conf;
    timer_conf.speed_mode       = LEDC_LOW_SPEED_MODE;
    timer_conf.duty_resolution  = LEDC_TIMER_16_BIT;
    timer_conf.timer_num        = LEDC_TIMER_0;
    timer_conf.freq_hz          = 1000;

    res = ledc_timer_config(&timer_conf);

    while(1);
}

extern "C" {
    int app_main(void){
        main();
    }
}
