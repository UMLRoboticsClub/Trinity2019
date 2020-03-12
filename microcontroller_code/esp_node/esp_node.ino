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

void solenoidCallback(const solenoidSrv::Request& req, solenoidSrv::Response& resp){
  digitalWrite(SOLENOID_PIN, req.trigger ? HIGH : LOW);  
}
ros::ServiceServer<SolenoidSrv::Request, solenoidSrv::Response> solenoidServer("solenoid_srv", solenoidCallback);

void irCallback(const irSrv::Request& req, irSrv resp){
  resp.reading = digitalRead(IR_PIN);  
}
ros::ServiceServer<irSrv::Request, irSrv::Response> irServer("ir_server", irCallback);

bool inRoom;
void inRoomCallback(const inRoomSrv::Request& req, inRoomSrv::Response& resp){
  resp.inRoom = inRoom;
}

ros::ServiceServer<inRoomSrv::Request, inRoomSrv::Response> inRoomServer("inRoom_server", inRoomCallback);



void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(encPub0);
  nh.advertise(encPub1);
  nh.advertise(encPub2);
  nh.advertise(encPub3);
  nh.subscribe(twistSub);
  
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
