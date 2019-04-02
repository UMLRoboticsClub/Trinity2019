#ifndef __FXAS21002C_H__
#define __FXAS21002C_H__

//Enum to define valid gyroscope range values
typedef enum {
	Range_250DPS = 0, Range_500DPS, Range_1000DPS, Range_2000DPS
} Range_t;

//250, 500, 1000, 2000
const uint8_t range_ctrl_reg[]  = { 0x03, 0x02, 0x01, 0x00 };
const float range_sensitivity[] = { 0.0078125f, 0.015625f, 0.03125f, 0.0625f };

//Raw register addresses used to communicate with the sensor.
typedef enum {
	GYRO_REGISTER_STATUS    = 0x00, /**< 0x00 */
	GYRO_REGISTER_OUT_X_MSB = 0x01, /**< 0x01 */
	GYRO_REGISTER_OUT_X_LSB = 0x02, /**< 0x02 */
	GYRO_REGISTER_OUT_Y_MSB = 0x03, /**< 0x03 */
	GYRO_REGISTER_OUT_Y_LSB = 0x04, /**< 0x04 */
	GYRO_REGISTER_OUT_Z_MSB = 0x05, /**< 0x05 */
	GYRO_REGISTER_OUT_Z_LSB = 0x06, /**< 0x06 */
	GYRO_REGISTER_WHO_AM_I  = 0x0C, /**< 0x0C (default value = 0b11010111, read only) */
	GYRO_REGISTER_CTRL_REG0 = 0x0D, /**< 0x0D (default value = 0b00000000, read/write) */
	GYRO_REGISTER_CTRL_REG1 = 0x13, /**< 0x13 (default value = 0b00000000, read/write) */
	GYRO_REGISTER_CTRL_REG2 = 0x14, /**< 0x14 (default value = 0b00000000, read/write) */
} gyroRegisters_t;

typedef struct {
	int16_t x, y, z;
} gyroData;

//Sensor driver for the Adafruit FXAS21002C breakout.
class FXAS21002C {
	public:
		FXAS21002C(const char *device, Range_t range = Range_250DPS);
        //update gyro values
        void read();

		gyroData raw, data;

	private:
		void write8(uint8_t addr, uint8_t value);
		uint8_t read8(uint8_t addr);
		Range_t range;
};

#endif

/*     Set CTRL_REG1 (0x13)
	   ====================================================================
	   BIT  Symbol    Description                                   Default
	   ---  ------    --------------------------------------------- -------
	   6  RESET     Reset device on 1                                   0
	   5  ST        Self test enabled on 1                              0
       4:2  DR        Output data rate                                  000
       000 = 800 Hz
       001 = 400 Hz
       010 = 200 Hz
       011 = 100 Hz
       100 = 50 Hz
       101 = 25 Hz
       110 = 12.5 Hz
       111 = 12.5 Hz
       1  ACTIVE    Standby(0)/Active(1)                                0
       0  READY     Standby(0)/Ready(1)                                 0
*/
       
/*     Set CTRL_REG0 (0x0D)  Default value 0x00
       =====================================================================
       BIT  Symbol     Description                                   Default
       7:6  BW         cut-off frequency of low-pass filter               00
       5  SPIW       SPI interface mode selection                        0
       4:3  SEL        High-pass filter cutoff frequency selection        00
       2  HPF_EN     High-pass filter enable                             0
       1:0  FS         Full-scale range selection
       00 = +-2000 dps
       01 = +-1000 dps
       10 = +-500 dps
       11 = +-250 dps
       The bit fields in CTRL_REG0 should be changed only in Standby or Ready modes.
*/

/*!
 * @file FXAS21002C.h
 *
 * This is part of Adafruit's FXAS21002C driver for the Arduino platform.  It
 * is designed specifically to work with the Adafruit FXAS21002C breakout:
 * https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

