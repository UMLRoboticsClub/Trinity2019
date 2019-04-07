#include <driver/i2c.h>
#include <esp_log.h>

#include "i2c.h"

static const char* TAG = "i2c";

void i2c_slave_init(uint8_t address){
    int i2c_slave_port = I2C_NUM_0;
    ESP_LOGI(TAG, "Starting I2C slave 1 at port %d.", i2c_slave_port);

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
