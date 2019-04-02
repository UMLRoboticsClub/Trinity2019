#include "FXAS21002C.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <climits>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

/** 7-bit address for this sensor */
#define FXAS21002C_ADDRESS       (0x21)       // 0100001
/** Device ID for this sensor (used as a sanity check during init) */
#define FXAS21002C_ID            (0xD7)       // 1101 0111

using std::cout;
using std::cerr;
using std::endl;

inline void sleep(int millis){
	std::this_thread::sleep_for(std::chrono::milliseconds(millis));
}

/*!
  @brief  Abstract away platform differences in Arduino wire library
  @param reg The register address to write to
  @param value The value to write to the specified register
  */
void FXAS21002C::write8(uint8_t addr, uint8_t value) {
    uint8_t packet[2] = { addr, d };
    //if(write(fd, packet, 2) != 2){ cerr << "Write failed" << endl; }
    while(write(fd, packet, 2) != 2){ cerr << "write failed(1)" << endl; }
}

/*!
  @brief  Abstract away platform differences in Arduino wire library
  @param reg The register address to read from
  @returns The uint8_t read from the I2C bus at the specified reg
  */
uint8_t FXAS21002C::read8(uint8_t addr) {
    while(write(fd, &addr, 1) != 1){ cerr << "write failed(2)" << endl; }
    while(read(fd, &addr, 1)  != 1){ cerr << "read failed(2)"  << endl; }

    //if(write(fd, &addr, 1) != 1){ cerr << "Read failed(1)" << endl; }
    //if(read(fd, &addr, 1)  != 1){ cerr << "Read failed(2)" << endl; }
    return addr;
}

/**************************************************************************/
/*!
  @brief  Instantiates a new FXAS21002C class, including assigning
  a unique ID to the gyroscope for logging purposes.
  */
/**************************************************************************/
FXAS21002C::FXAS21002C(const char *device, Range_t range): range(range) {
    if((fd = open(device, O_RDWR)) < 0){ cerr << "Unable to open I2C device" << endl; }
    if(ioctl(fd, I2C_SLAVE, FXAS21002C_ADDRESS) < 0){ cerr << "Unable to connect to I2C device" << endl; }

	/* Make sure we have the correct chip ID since this checks
	   for correct address and that the IC is properly connected */
	if (read8(GYRO_REGISTER_WHO_AM_I) != FXAS21002C_ID) {
        cerr << "Unable to connect to I2C device" << endl;
		exit(1);
	}

    /* Reset then switch to active mode with 100Hz output */
    //see FXAS21002C.h for more info on CTRL_REG
    write8(GYRO_REGISTER_CTRL_REG1, 0x00); // Standby
    write8(GYRO_REGISTER_CTRL_REG1, 1<<6); // Reset
    write8(GYRO_REGISTER_CTRL_REG0, range_ctrl_reg[range]); // Set sensitivity
    write8(GYRO_REGISTER_CTRL_REG1, 0x0E); // Active
    sleep(100); // 60 ms + 1/ODR
}

void FXAS21002C::read(){
    /* Read 7 uint8_ts from the sensor */
    union {
        struct {
            uint8_t status;
            uint8_t xhi;
            uint8_t xlo;
            uint8_t yhi;
            uint8_t ylo;
            uint8_t zhi;
            uint8_t zlo;
        };
        uint8_t data[7];
    };

    uint8_t addr = GYRO_REGISTER_STATUS | 0x80;
    while(write(fd, &addr, 1) != 1){ cerr << "write failed(2)" << endl; }
    while(read(fd, &data, 7)  != 7){ cerr << "read failed(2)"  << endl; }

    /* Shift values to create properly formed integer */
    data = raw = {
        (xhi << 8) | xlo, //x
        (yhi << 8) | ylo, //y
        (zhi << 8) | zlo  //z
    };

    /* Compensate values depending on the resolution */
    data.x *= range_sensitivity[range];
    data.y *= range_sensitivity[range];
    data.z *= range_sensitivity[range];

    /* Convert values to rad/s */
    event->gyro.x *= SENSORS_DPS_TO_RADS;
    event->gyro.y *= SENSORS_DPS_TO_RADS;
    event->gyro.z *= SENSORS_DPS_TO_RADS;
}

/*!
 * @file FXAS21002C.cpp
 *
 * @mainpage Adafruit FXAS21002C gyroscope sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's FXAS21002C driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit FXAS21002C breakout: https://www.adafruit.com/products/3463
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
