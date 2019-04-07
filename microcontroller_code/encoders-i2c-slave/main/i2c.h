#pragma once

//#define SPEKTRUM_DATA_LENGTH    16

#define SLAVE_1_DATA_LENGTH     1 /* One byte. */

#define I2C_SLAVE_1_SCL         19 /* Use yellow wire. */
#define I2C_SLAVE_1_SDA         18 /* Use green wire. */
//#define I2C_SLAVE_1_NUM         I2C_NUM_0
#define I2C_SLAVE_1_TX_BUF_LEN  (1024 * SLAVE_1_DATA_LENGTH)
#define I2C_SLAVE_1_RX_BUF_LEN  (1024 * SLAVE_1_DATA_LENGTH)

//#define I2C_SLAVE_2_SCL         26 /* Use yellow wire. */
//#define I2C_SLAVE_2_SDA         25 /* Use green wire. */
//#define I2C_SLAVE_2_NUM         I2C_NUM_1
//#define I2C_SLAVE_2_TX_BUF_LEN  (1024 * SLAVE_2_DATA_LENGTH)
//#define I2C_SLAVE_2_RX_BUF_LEN  (1024 * SLAVE_1_DATA_LENGTH)

//#define ACK_CHECK_ENABLE        0x1 /* Master will require ack from slave */
//#define ACK_CHECK_DISABLE       0x0
//#define ACK_VAL                 0x0
//#define NACK_VAL                0x1

void i2c_slave_init(uint8_t address);
