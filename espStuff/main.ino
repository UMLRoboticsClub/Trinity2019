#include "drive.h"



void irSrvCallback(const IrSrv::Request& req, IrSrv::Response& resp){
  resp.reading = digitalRead(IR_PIN);
}

void solenoidSrvCallback(const SolenoidSrv::Request& req, SolenoidSrv::Response& resp){
  if(req.activate)
    digitalWrite(SOLENOID_PIN, HIGH)
  else
    digitalWrite(SOLENOID_PIN, LOW)
}



int main(int argc, char* argv[]){

    ros::init(argc, argv, "esp");
    ros::NodeHandle nh;

    //initialize publishers
    ros::Publisher encPub0 = nh.advertise<std_msgs::Int32>("encoder0", 1000);
    ros::Publisher encPub1 = nh.advertise<std_msgs::Int32>("encoder1", 1000);
    ros::Publisher encPub2 = nh.advertise<std_msgs::Int32>("encoder2", 1000);
    ros::Publisher encPub3 = nh.advertise<std_msgs::Int32>("encoder3", 1000);

    //initialize services
    ros::ServiceServer<IrSrv::Request, IrSrv::Response> irServer("IrSrv", &irSrvCallback);
    ros::ServiceServer<SolenoidSrv::Request, SolenoidSrv::Response> solenoidServer("SolenoidSrv", &solenoidSrvCallback);

    //initialize subscribers
    ros::subscriber twistSub = nh.subscribe("MotorTwist", 1000, twistToMotors);

    Encoder encoders[4];
    encoders[0] = {ENCODER_PIN_0, 0};
    encoders[1] = {ENCODER_PIN_1, 0};
    encoders[2] = {ENCODER_PIN_2, 0};
    encoders[3] = {ENCODER_PIN_3, 0};

    for(int i = 0; i < 4; i ++){
        attachInterruptArg(encoders[i].PIN, encoderInterrupt, encoders + i, CHANGE);
        pinMode(encoders[i].PIN, INPUT_PULLUP);
    }

    motorControlInitialize();
    double lastReflectanceReading, newReflectanceReading;
    int lightTicks = 0;
    int darkTicks = 0;

    bool inRoom = true;//maybe?
    bool toggled = false;
    ros::Rate loopRate(200);
    while(ros::ok()){
        encPub0.publish(std_msgs::Int32{encoders[0].ticks});
        encPub1.publish(std_msgs::Int32{encoders[1].ticks});
        encPub2.publish(std_msgs::Int32{encoders[2].ticks});
        encPub3.publish(std_msgs::Int32{encoders[3].ticks});

        // in this loop check for reflectance sensor to track if we are in a room
        // and also check for microphone stuff.  If yes, publish to microphone.
        newReflectanceReading = analogRead(reflectancePin);
        if (newReflectanceReading > TAPE_THRESHOLD){
          lightTicks ++;
          if (lightTicks > TAPE_TICKS_THRESHOLD){
            darkTicks = 0;
            if(!toggled){
              darkTicks = 0;
              inRoom != inRoom;
              toggled = true;
            }
          }
        }
        else{
          darkTicks ++;
          if(darkTicks > TAPE_TICKS_THRESHOLD){
            lightTicks = 0;
            toggled = false;
          }
        }

        ros::spinOnce();
        loopRate.sleep();
    }
}
