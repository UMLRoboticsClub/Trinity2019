#pragma once

#include <cstdint>

//min_delay   = 0.01F;
//max_value   = 1200.0F;
//min_value   = -1200.0F;
//resolution  = 0.1F;

//Range settings for the accelerometer sensor.
typedef enum {
    ACCEL_RANGE_2G = 0x00, /**< +/- 2g range */
    ACCEL_RANGE_4G = 0x01, /**< +/- 4g range */
    ACCEL_RANGE_8G = 0x02  /**< +/- 8g range */
} Range;

typedef struct {
    int16_t x, y, z;
} fxosData;

class FXOS8700 {
    public:
        FXOS8700(const char *device, Range range = ACCEL_RANGE_2G);
        ~FXOS8700();

        //update sensor values
        void update();
        //puts device into/out of standby mode
        void standby(bool standby);

        //Raw accelerometer/magnetometer values from last read
        fxosData accel, accel_raw, mag, mag_raw;

    private:
        int fd;
        void write8(uint8_t addr, uint8_t val);
        uint8_t read8(uint8_t addr);

        Range range;
};

/*!
 * @file FXOS8700.h
 *
 * This is part of Adafruit's FXOS8700 driver for the Arduino platform.  It is
 * designed specifically to work with the Adafruit FXOS8700 breakout:
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
