#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"

#define turnRadius 124 //mm I guess?
#define wheelRadius 42.94
#define ENCODER_PIN_0 35
#define ENCODER_PIN_1 34
#define ENCODER_PIN_2 39
#define ENCODER_PIN_3 36

#define MOTOR_PIN_0 16
#define MOTOR_PIN_1 17
#define MOTOR_PIN_2 18
#define MOTOR_PIN_3 19

#define MOTOR_DIR_0 27
#define MOTOR_DIR_1 26
#define MOTOR_DIR_2 33
#define MOTOR_DIR_3 32

#define PWM_FREQ 240
#define PWM_RESOLUTION 10

struct Encoder {
    const int PIN;
    int ticks;
};

motorControlInitialize();
int initialMotorPWM(double motorSpeed);
void IRAM_ATTR encoderInterrupt(void* encoder);
void twistToMotors(const geometry_msgs::Twist twist);
