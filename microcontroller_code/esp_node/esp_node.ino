#include "drive.h"

ros::NodeHandle nh;
Encoder encoders[] = {{ENCODER_PIN_0, std_msgs::Int32()},
                       {ENCODER_PIN_1, std_msgs::Int32()},
                       {ENCODER_PIN_2, std_msgs::Int32()},
                       {ENCODER_PIN_3, std_msgs::Int32()}};
    
ros::Publisher encPub0("encoder0", &encoders[0].ticks);
ros::Publisher encPub1("encoder1", &encoders[1].ticks);
ros::Publisher encPub2("encoder2", &encoders[2].ticks);
ros::Publisher encPub3("encoder3", &encoders[3].ticks);

ros::Subscriber<geometry_msgs::Twist> twistSub("motorTwist", &twistToMotors);

void setup() {
  // put your setup code here, to run once:
  nh.initNode();

  for(int i = 0; i < 4; i ++){
    attachInterruptArg(encoders[i].PIN, encoderInterrupt, encoders + i, FALLING);
    pinMode(encoders[i].PIN, INPUT_PULLUP);
  }

  motorControlInitialize();
}

void loop() {
  // put your main code here, to run repeatedly:
  encPub0.publish(&encoders[0].ticks);
  encPub1.publish(&encoders[1].ticks);
  encPub2.publish(&encoders[2].ticks);
  encPub3.publish(&encoders[3].ticks);
  nh.spinOnce();
  delay(5);
}
