# Trinity2019
This is the UML Robotics Club's code for the [2019 Trinity College International Firefighting Robot Contest](https://trinityrobotcontest.org/).

To build this, first create a catkin workspace, then clone this repo into the `src` directory. Then call `catkin build` to compile the project.


Our project uses ROS, and is designed to run on a Raspberry Pi and communicate with a host computer. Our peripherals include:
* RPLidar A1 lidar sensor to create a map of the maze
* IR Flame sensor to detect when the robot is looking at a candle
* Versa EZ Valve to release CO2 to put out the flame
* Bosch BNO055 IMU to get acceleration and gyroscope information
* An Adafruit color sensor to detect when our robot enters rooms
* Servos to grab and lift the cradle
* 3x Vex omni wheels and geared hobby motors with encoders
* Custom-made logic board and motor driver board

## ROS Packages Used
* [RPLidar Node](https://github.com/Slamtec/rplidar_ros/)
* [BNO055 Node](https://github.com/dheera/ros-imu-bno055/)
* A heavily modified version of jfstepha's [differential-drive](https://github.com/jfstepha/differential-drive) to work with a kiwi drive configuration
