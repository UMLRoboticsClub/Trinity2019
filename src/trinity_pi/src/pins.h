#pragma once
/* Pi GPIOs*/

//motor A encoder
#define MOTORA_ENCA 4
#define MOTORA_ENCB 17
//motor B encoder
#define MOTORB_ENCA 27
#define MOTORB_ENCB 22
//motor C encoder
#define MOTORC_ENCA 10
#define MOTORC_ENCB 9

//ir sensor
#define IR_SENSOR   11
//co2 solenoid
#define SOLENOID    5
//servo, hardware PWM pin
#define SERVO       13
//status LED
#define LED_STATUS  18

//I2C bus (for reference)
//#define SDA       2
//#define SCL       3

/* PCA9685 pins */

//motor A
#define PCA_0  0
#define PCA_1  1
//motor B
#define PCA_2  3
#define PCA_3  2
//motor C
#define PCA_4  4
#define PCA_5  5

#define PCA_6  6
#define PCA_7  7
#define PCA_8  8
#define PCA_9  9
#define PCA_10 10
#define PCA_11 11
#define PCA_12 12
#define PCA_13 13
#define PCA_14 14
#define PCA_15 15

//motor control pins
#define MOTORA_PINA PCA_0
#define MOTORA_PINB PCA_1
#define MOTORB_PINA PCA_2
#define MOTORB_PINB PCA_3
#define MOTORC_PINA PCA_4
#define MOTORC_PINB PCA_5
//LEDs
#define LED_FIRE    PCA_7
#define LED_VIDEO   PCA_8
#define LED_SND_ACT PCA_9

//blink all the leds when program started
