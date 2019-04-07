#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <esp_log.h>
#include <driver/i2c.h>
#include <time.h>
#include <stdlib.h>
#include <esp_task_wdt.h>

#include "sdkconfig.h"
#include "i2c.h"

#define ENC1_A GPIO_NUM_27
#define ENC1_B GPIO_NUM_14
#define ENC2_A GPIO_NUM_22
#define ENC2_B GPIO_NUM_23
#define ENC3_A GPIO_NUM_4
#define ENC3_B GPIO_NUM_21

union Data {
    int32_t counter[3];
    uint8_t buf[4*3];
} d;

void IRAM_ATTR enc1_a(void *arg){ gpio_get_level(ENC1_A) == gpio_get_level(ENC1_B) ? --d.counter[0] : ++d.counter[0]; }
void IRAM_ATTR enc1_b(void *arg){ gpio_get_level(ENC1_A) == gpio_get_level(ENC1_B) ? ++d.counter[0] : --d.counter[0]; }
void IRAM_ATTR enc2_a(void *arg){ gpio_get_level(ENC2_A) == gpio_get_level(ENC2_B) ? --d.counter[1] : ++d.counter[1]; }
void IRAM_ATTR enc2_b(void *arg){ gpio_get_level(ENC2_A) == gpio_get_level(ENC2_B) ? ++d.counter[1] : --d.counter[1]; }
void IRAM_ATTR enc3_a(void *arg){ gpio_get_level(ENC3_A) == gpio_get_level(ENC3_B) ? --d.counter[2] : ++d.counter[2]; }
void IRAM_ATTR enc3_b(void *arg){ gpio_get_level(ENC3_A) == gpio_get_level(ENC3_B) ? ++d.counter[2] : --d.counter[2]; }

//already set to 240mHz again

/*
#define NTOHL(n) ((((n & 0xFF))       << 24) | \
              (((n & 0xFF00))     << 8)  | \
              (((n & 0xFF0000))   >> 8)  | \
              (((n & 0xFF000000)) >> 24))
*/

void i2c_slave_task(void *params){
    esp_task_wdt_deinit();

    uint8_t addr;
    while(1){

        if(i2c_slave_read_buffer(I2C_NUM_0, &addr, 1, 0) > 0){
            uint8_t packet = 0xFF;
            switch(addr){
                case 0x05:
                    i2c_slave_write_buffer(I2C_NUM_0, d.buf, 12, 1000);
                    break;
                case 0x06:
                    i2c_slave_write_buffer(I2C_NUM_0, &addr, 1, 1000);
                    break;
                default:
                    i2c_slave_write_buffer(I2C_NUM_0, &packet, 1, 1000);
            }
        }
        vTaskDelay(5 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void printy(void *arg){
     while(1){
        printf("%d, %d, %d\n", d.counter[0], d.counter[1], d.counter[2]);
        vTaskDelay(200 / portTICK_RATE_MS);
     }
}

void app_main(){
    memset(d.counter, 0, 3*sizeof(uint32_t));

    ESP_ERROR_CHECK(gpio_set_intr_type(ENC1_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENC1_B, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENC2_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENC2_B, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENC3_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_set_intr_type(ENC3_B, GPIO_INTR_ANYEDGE));

    ESP_ERROR_CHECK(gpio_intr_enable(ENC1_A));
    ESP_ERROR_CHECK(gpio_intr_enable(ENC1_B));
    ESP_ERROR_CHECK(gpio_intr_enable(ENC2_A));
    ESP_ERROR_CHECK(gpio_intr_enable(ENC2_B));
    ESP_ERROR_CHECK(gpio_intr_enable(ENC3_A));
    ESP_ERROR_CHECK(gpio_intr_enable(ENC3_B));

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC1_A, enc1_a, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC1_B, enc1_b, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC2_A, enc2_a, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC2_B, enc2_b, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC3_A, enc3_a, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(ENC3_B, enc3_b, NULL));

    i2c_slave_init(0x03);
    xTaskCreatePinnedToCore(i2c_slave_task, "I2C", 2048, NULL, 2, NULL, 1);

    //xTaskCreatePinnedToCore(printy, "printy", 2048, NULL, 2, NULL, 1);
}
