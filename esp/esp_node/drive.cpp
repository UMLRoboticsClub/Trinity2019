#include "drive.h"

void motorControlInitialize(){
    pinMode(MOTOR_PIN_0, OUTPUT);
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    pinMode(MOTOR_PIN_3, OUTPUT);

    pinMode(MOTOR_DIR_0, OUTPUT);
    pinMode(MOTOR_DIR_1, OUTPUT);
    pinMode(MOTOR_DIR_2, OUTPUT);
    pinMode(MOTOR_DIR_3, OUTPUT);

    ledcSetup(MOTOR_PIN_0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(MOTOR_PIN_1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(MOTOR_PIN_2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(MOTOR_PIN_3, PWM_FREQ, PWM_RESOLUTION);


    ledcWrite(MOTOR_PIN_0, 0);
    ledcWrite(MOTOR_PIN_1, 0);
    ledcWrite(MOTOR_PIN_2, 0);
    ledcWrite(MOTOR_PIN_3, 0);
}


void twistToMotorVels(const geometry_msgs::Twist& robotTwist, double* motorVels){
    motorVels[2] = robotTwist.linear.y;
    motorVels[0] = -robotTwist.linear.y;
    motorVels[3] = -robotTwist.linear.x;
    motorVels[1] = robotTwist.linear.x;
    double rotLinVel = -turnRadius*robotTwist.angular.z;

    for(int i = 0; i < 4; i ++){
        motorVels[i] += rotLinVel;
        motorVels[i] /= wheelRadius;
    }
}

int initialMotorPWM(double motorSpeed){
    return static_cast<int>((motorSpeed+50.804)/56.558/5*1024);
}

void IRAM_ATTR encoderInterrupt(void* encoder){
    Encoder* enc = static_cast<Encoder*>(encoder);
    enc->ticks.data += 1;
}

void driveMotor(int motorPin, int directionPin, int pwm){
	if (pwm < 0){
		digitalWrite(directionPin, LOW);
		pwm = -pwm;
	}
	else{
		digitalWrite(directionPin, HIGH);
	}
	ledcWrite(motorPin, pwm);
}

void twistToMotors(const geometry_msgs::Twist& twist){
    double motorVels[4];
    int outputPWM[4];
    twistToMotorVels(twist, motorVels);

    for(int i = 0; i < 4; i++){
        outputPWM[i] = initialMotorPWM(motorVels[i]);
    }
    
    driveMotor(MOTOR_PIN_0, MOTOR_DIR_0, outputPWM[0]);
    driveMotor(MOTOR_PIN_1, MOTOR_DIR_1, outputPWM[1]);
    driveMotor(MOTOR_PIN_2, MOTOR_DIR_2, outputPWM[2]);
    driveMotor(MOTOR_PIN_3, MOTOR_DIR_3, outputPWM[3]);
}
