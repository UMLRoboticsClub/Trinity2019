#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#define turnRadius 124
#define wheelRadius 42.94
#define ENCODER_PIN_0 34
#define ENCODER_PIN_1 35
#define ENCODER_PIN_2 36
#define ENCODER_PIN_3 39

#define MOTOR_PIN_0 16
#define MOTOR_PIN_1 17
#define MOTOR_PIN_2 18
#define MOTOR_PIN_3 19

#define MOTOR_DIR_0 26
#define MOTOR_DIR_1 27
#define MOTOR_DIR_2 32
#define MOTOR_DIR_3 33

#define PWM_FREQ 240
#define PWM_RESOLUTION 10

struct Encoder {
    const int PIN;
    std_msgs::Int32 ticks;
};

void motorControlInitialize();
int initialMotorPWM(double motorSpeed);
void IRAM_ATTR encoderInterrupt(void* encoder);
void twistToMotors(const geometry_msgs::Twist& twist);
