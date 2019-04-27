#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <esp_log.h>
#include <driver/i2c.h>
#include <time.h>
#include <stdlib.h>
#include <esp_task_wdt.h>

#include <driver/i2c.h>
#include <esp_log.h>

#include "sdkconfig.h"

#define REG_GETVAL   0x05 
#define REG_ECHO     0x06 
#define REG_CLEARVAL 0x07 
#define REG_RESET    0x08 

#define I2C_SLAVE_1_SCL         19
#define I2C_SLAVE_1_SDA         18
#define I2C_SLAVE_1_TX_BUF_LEN  (1024)
#define I2C_SLAVE_1_RX_BUF_LEN  (1024)

#define ENC1_A GPIO_NUM_27
#define ENC1_B GPIO_NUM_14
#define ENC2_A GPIO_NUM_22
#define ENC2_B GPIO_NUM_23
#define ENC3_A GPIO_NUM_21
#define ENC3_B GPIO_NUM_4
//#define I2C_SLAVE_1_SCL 19
//#define I2C_SLAVE_1_SDA 18

int count1 = 0;
int count2 = 0;
int count3 = 0;

union Data {
    int32_t counter[3];
    uint8_t buf[4*3];
} d;

void IRAM_ATTR enc1_a(void *arg){ count1++; gpio_get_level(ENC1_A) == gpio_get_level(ENC1_B) ? --d.counter[0] : ++d.counter[0];}
void IRAM_ATTR enc1_b(void *arg){ count1++; gpio_get_level(ENC1_A) == gpio_get_level(ENC1_B) ? ++d.counter[0] : --d.counter[0];}
void IRAM_ATTR enc2_a(void *arg){ count2++; gpio_get_level(ENC2_A) == gpio_get_level(ENC2_B) ? --d.counter[1] : ++d.counter[1]; }
void IRAM_ATTR enc2_b(void *arg){ count2++; gpio_get_level(ENC2_A) == gpio_get_level(ENC2_B) ? ++d.counter[1] : --d.counter[1]; }
void IRAM_ATTR enc3_a(void *arg){ count3++; gpio_get_level(ENC3_A) == gpio_get_level(ENC3_B) ? --d.counter[2] : ++d.counter[2];}
void IRAM_ATTR enc3_b(void *arg){ count3++; gpio_get_level(ENC3_A) == gpio_get_level(ENC3_B) ? ++d.counter[2] : --d.counter[2];}

void i2c_slave_init(uint8_t address){
    int i2c_slave_port = I2C_NUM_0;
    ESP_LOGI("i2c", "Starting I2C slave 1 at port %d.", i2c_slave_port);

    i2c_config_t conf;
    conf.sda_io_num          = I2C_SLAVE_1_SDA;
    conf.sda_pullup_en       = GPIO_PULLUP_ENABLE;
    conf.scl_io_num          = I2C_SLAVE_1_SCL;
    conf.scl_pullup_en       = GPIO_PULLUP_ENABLE;
    conf.mode                = I2C_MODE_SLAVE;
    conf.slave.addr_10bit_en = 0;
    conf.slave.slave_addr    = address;

    ESP_ERROR_CHECK(i2c_param_config(i2c_slave_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(
        i2c_slave_port,
        conf.mode,
        I2C_SLAVE_1_RX_BUF_LEN,
        I2C_SLAVE_1_TX_BUF_LEN,
        0
    ));
}

void i2c_slave_uninit(){
    ESP_ERROR_CHECK(i2c_driver_delete(I2C_NUM_0));
}

void i2c_slave_task(void *params){
    esp_task_wdt_deinit();

    uint8_t startByte = 170;
    uint8_t endByte   = 169;

    const int waitTicks = 1000;

    uint8_t addr;
    while(1){

        if(i2c_slave_read_buffer(I2C_NUM_0, &addr, 1, 0) > 0){
            uint8_t packet = 0xFF;
            switch(addr){
                case REG_GETVAL:
                    i2c_slave_write_buffer(I2C_NUM_0, &startByte, 1, waitTicks);
					//ets_printf("%d\n", startByte);
                    i2c_slave_write_buffer(I2C_NUM_0, d.buf, 12, waitTicks);
					//for(int i = 0; i < 12; i++)
					//	ets_printf("%d, ", d.buf[i]);
                    i2c_slave_write_buffer(I2C_NUM_0, &endByte, 1, waitTicks);
					//ets_printf("\n%d\n", endByte);
                    break;
                case REG_ECHO:
                    i2c_slave_write_buffer(I2C_NUM_0, &addr, 1, waitTicks);
                    break;
                case REG_CLEARVAL:
                    d.counter[0] = d.counter[1] = d.counter[2] = 0;
                    break;
                case REG_RESET:
                    i2c_slave_uninit();
                    vTaskDelay(2 / portTICK_RATE_MS);
                    i2c_slave_init(0x03);
                    break;
                default:
                    i2c_slave_write_buffer(I2C_NUM_0, &packet, 1, waitTicks);
            }
        }
        vTaskDelay(2 / portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void printy(void *arg){
    uint8_t startByte = 63;
    uint8_t endByte   = 64;
    while(1){
		//char c = getchar();
		//printf(" %d ", c);
		if(getchar() != -1){
			printf("%c", startByte);
			printf("%c%c%c%c%c%c%c%c%c%c%c%c", d.buf[0], d.buf[1], d.buf[2], d.buf[3], d.buf[4], d.buf[5], d.buf[6], d.buf[7], d.buf[8], d.buf[9], d.buf[10], d.buf[11]);
			printf("%c", endByte);
		}
        //printf("%d, %d, %d, %d, %d, %d\n", d.counter[0], d.counter[1], d.counter[2], count1, count2, count3);
        //printf("1A:%d, 1B:%d\n", gpio_get_level(ENC1_A), gpio_get_level(ENC1_B));
        //printf("2A:%d, 2B:%d\n", gpio_get_level(ENC2_A), gpio_get_level(ENC2_B));
        //printf("3A:%d, 3B:%d\n", gpio_get_level(ENC3_A), gpio_get_level(ENC3_B));
        //vTaskDelay(200 / portTICK_RATE_MS);
   }
}

void init_gpio(void *params){
    ESP_ERROR_CHECK(gpio_set_direction(ENC1_A, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(ENC1_B, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(ENC2_A, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(ENC2_B, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(ENC3_A, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction(ENC3_B, GPIO_MODE_INPUT));

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

    while(1){ vTaskDelay(100000 / portTICK_RATE_MS); }
}

void app_main(){
    d.counter[0] = d.counter[1] = d.counter[2] = 0xcccccccc;
	//init_gpio(NULL);
    xTaskCreatePinnedToCore(init_gpio, "init_gpio", 1024, NULL, 1, NULL, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    //i2c_slave_init(0x03);
    //xTaskCreatePinnedToCore(i2c_slave_task, "i2c_slave", 1024, NULL, 2, NULL, 0);

    xTaskCreatePinnedToCore(printy, "printy", 2048, NULL, 2, NULL, 1);
}
