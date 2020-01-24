#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"

#define turnRadius 124
#define wheelRadius 42.94
#define ENCODER_PIN_0 0
#define ENCODER_PIN_1 1
#define ENCODER_PIN_2 2
#define ENCODER_PIN_3 3

#define MOTOR_PIN_0 4
#define MOTOR_PIN_1 5
#define MOTOR_PIN_2 6
#define MOTOR_PIN_3 7

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
