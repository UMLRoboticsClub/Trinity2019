#include "drive.h"

int main(int argc, char* argv[]){

    ros::init(argc, argv, "esp");
    ros::NodeHandle nh;

    ros::Publisher encPub0 = nh.advertise<std_msgs::Int32>("encoder0", 1000);
    ros::Publisher encPub1 = nh.advertise<std_msgs::Int32>("encoder1", 1000);
    ros::Publisher encPub2 = nh.advertise<std_msgs::Int32>("encoder2", 1000);
    ros::Publisher encPub3 = nh.advertise<std_msgs::Int32>("encoder3", 1000);

    ros::subscriber twistSub = nh.subscribe("MotorTwist", 1000, twistToMotors);

    Encoder encoders[4];
    encoders[0] = {ENCODER_PIN_0, 0};
    encoders[1] = {ENCODER_PIN_1, 0};
    encoders[2] = {ENCODER_PIN_2, 0};
    encoders[3] = {ENCODER_PIN_3, 0};

    for(int i = 0; i < 4; i ++){
        attachInterruptArg(encoders[i].PIN, encoderInterrupt, encoders + i, FALLING);
        pinMode(encoders[i].PIN, INPUT_PULLUP);
    }

    motorControlInitialize();

    ros::Rate loopRate(200);
    while(ros::ok()){
        encPub0.publish(std_msgs::Int32{encoders[0].ticks});
        encPub1.publish(std_msgs::Int32{encoders[1].ticks});
        encPub2.publish(std_msgs::Int32{encoders[2].ticks});
        encPub3.publish(std_msgs::Int32{encoders[3].ticks});

        ros::spinOnce();
        loopRate.sleep();
    }
}
