#include "fxos8700.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <climits>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

/** 7-bit I2C address for this sensor */
#define FXOS8700_ADDRESS           (0x1F)     // 0011111
/** Device ID for this sensor (used as sanity check during init) */
#define FXOS8700_ID                (0xC7)     // 1100 0111

/** Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT) */
#define MAG_UT_LSB      (0.1F)

static const float mg_lsb[] = { 0.000244f, 0.000488f, 0.000976f };

using std::cout;
using std::cerr;
using std::endl;

//Raw register addresses used to communicate with the sensor.
typedef enum {
    FXOS8700_REGISTER_STATUS          = 0x00, /**< 0x00 */
    FXOS8700_REGISTER_OUT_X_MSB       = 0x01, /**< 0x01 */
    FXOS8700_REGISTER_OUT_X_LSB       = 0x02, /**< 0x02 */
    FXOS8700_REGISTER_OUT_Y_MSB       = 0x03, /**< 0x03 */
    FXOS8700_REGISTER_OUT_Y_LSB       = 0x04, /**< 0x04 */
    FXOS8700_REGISTER_OUT_Z_MSB       = 0x05, /**< 0x05 */
    FXOS8700_REGISTER_OUT_Z_LSB       = 0x06, /**< 0x06 */
    FXOS8700_REGISTER_WHO_AM_I        = 0x0D, /**< 0x0D (default value = 0b11000111, read only) */
    FXOS8700_REGISTER_XYZ_DATA_CFG    = 0x0E, /**< 0x0E */
    FXOS8700_REGISTER_CTRL_REG1       = 0x2A, /**< 0x2A (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_CTRL_REG2       = 0x2B, /**< 0x2B (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_CTRL_REG3       = 0x2C, /**< 0x2C (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_CTRL_REG4       = 0x2D, /**< 0x2D (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_CTRL_REG5       = 0x2E, /**< 0x2E (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_MSTATUS         = 0x32, /**< 0x32 */
    FXOS8700_REGISTER_MOUT_X_MSB      = 0x33, /**< 0x33 */
    FXOS8700_REGISTER_MOUT_X_LSB      = 0x34, /**< 0x34 */
    FXOS8700_REGISTER_MOUT_Y_MSB      = 0x35, /**< 0x35 */
    FXOS8700_REGISTER_MOUT_Y_LSB      = 0x36, /**< 0x36 */
    FXOS8700_REGISTER_MOUT_Z_MSB      = 0x37, /**< 0x37 */
    FXOS8700_REGISTER_MOUT_Z_LSB      = 0x38, /**< 0x38 */
    FXOS8700_REGISTER_MCTRL_REG1      = 0x5B, /**< 0x5B (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_MCTRL_REG2      = 0x5C, /**< 0x5C (default value = 0b00000000, read/write) */
    FXOS8700_REGISTER_MCTRL_REG3      = 0x5D, /**< 0x5D (default value = 0b00000000, read/write) */
} fxos8700Registers_t;

inline void sleep(int millis){
	std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

void FXOS8700::write8(uint8_t addr, uint8_t val) {
    uint8_t packet[2] = { addr, val };
    //if(write(fd, packet, 2) != 2){ cerr << "Write failed" << endl; }
    while(write(fd, packet, 2) != 2){ cerr << "write failed(1)" << endl; }
}

uint8_t FXOS8700::read8(uint8_t addr){
    while(write(fd, &addr, 1) != 1){ cerr << "write failed(2)" << endl; }
    while(read(fd, &addr, 1)  != 1){ cerr << "read failed(2)"  << endl; }

    //if(write(fd, &addr, 1) != 1){ cerr << "Read failed(1)" << endl; }
    //if(read(fd, &addr, 1)  != 1){ cerr << "Read failed(2)" << endl; }
    return addr;
}

FXOS8700::FXOS8700(const char *device, Range range): range(range){
	/* Enable I2C */
    if((fd = open(device, O_RDWR)) < 0){ cerr << "Unable to open I2C device" << endl; }
    if(ioctl(fd, I2C_SLAVE, FXOS8700_ADDRESS) < 0){ cerr << "Unable to connect to I2C device" << endl; }

	/* Make sure we have the correct chip ID since this checks
	   for correct address and that the IC is properly connected */
	if (read8(FXOS8700_REGISTER_WHO_AM_I) != FXOS8700_ID) {
        cerr << "Unable to connect to I2C device" << endl;
		exit(1);
	}

	/* Set to standby mode (required to make changes to this register) */
	write8(FXOS8700_REGISTER_CTRL_REG1, 0);

	/* Configure the accelerometer */
	switch (_range) {
		case (ACCEL_RANGE_2G):
			write8(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x00);
			break;
		case (ACCEL_RANGE_4G):
			write8(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x01);
			break;
		case (ACCEL_RANGE_8G):
			write8(FXOS8700_REGISTER_XYZ_DATA_CFG, 0x02);
			break;
	}
	/* High resolution */
	write8(FXOS8700_REGISTER_CTRL_REG2, 0x02);
	/* Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode */
	write8(FXOS8700_REGISTER_CTRL_REG1, 0x15);

	/* Configure the magnetometer */
	/* Hybrid Mode, Over Sampling Rate = 16 */
	write8(FXOS8700_REGISTER_MCTRL_REG1, 0x1F);
	/* Jump to reg 0x33 after reading 0x06 */
	write8(FXOS8700_REGISTER_MCTRL_REG2, 0x20);
}

FXOS8700::~FXOS8700(){
    close(fd);
}

bool FXOS8700::read(){
    /* Read 13 values from the sensor */
	/* Todo: Check status first! */
    union {
        struct {
            uint8_t
                status,
                axhi,
                axlo,
                ayhi,
                aylo,
                azhi,
                azlo,
                mxhi,
                mxlo,
                myhi,
                mylo,
                mzhi,
                mzlo;
        };
        uint8_t data[13];
    };

    uint8_t addr = FXOS8700_REGISTER_STATUS | 0x80;
    while(write(fd, &addr, 1) != 1){ cerr << "write failed(2)" << endl; }
    while(read(fd, &data, 13)  != 13){ cerr << "read failed(2)"  << endl; }

	/* Shift values to create properly formed integers */
	/* Note, accel data is 14-bit and left-aligned, so we shift two bit right */
    accel = accel_raw = {
        ((axhi << 8) | axlo) >> 2,
        ((ayhi << 8) | aylo) >> 2,
        ((azhi << 8) | azlo) >> 2
    };

    mag = mag_raw = {
        (mxhi << 8) | mxlo,
        (myhi << 8) | mylo,
        (mzhi << 8) | mzlo
    };

    /* Convert accel values to m/s^2 */
    accel.x *= mg_lsb[range] * SENSORS_GRAVITY_STANDARD;
    accel.y *= mg_lsb[range] * SENSORS_GRAVITY_STANDARD;
    accel.z *= mg_lsb[range] * SENSORS_GRAVITY_STANDARD;

    /* Convert mag values to uTesla */
    mag.x *= MAG_UT_LSB;
    mag.y *= MAG_UT_LSB;
    mag.z *= MAG_UT_LSB;
}

//standby: Set this to a non-zero value to enter standy mode.
void FXOS8700::standby(bool standby) {
    uint8_t reg1 = read8(FXOS8700_REGISTER_CTRL_REG1);
    if (standby) {
        reg1 &= ~(0x01);
    } else {
        reg1 |= (0x01);
    }
    write8(FXOS8700_REGISTER_CTRL_REG1, reg1);

    if (!standby) {
        sleep(100);
    }
}

/*!
 * @file FXOS8700.cpp
 *
 * @mainpage Adafruit FXOS8700 accel/mag sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's FXOS8700 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit FXOS8700 breakout: https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a href="https://github.com/adafruit/Sensor">
 * Sensor</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

