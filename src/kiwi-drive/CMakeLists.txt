cmake_minimum_required(VERSION 2.8.3)
project(kiwi-drive)

add_compile_options(
    -std=c++14
    -Wall
    -Wextra
    -Wfatal-errors
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs    
    sensor_msgs    
    geometry_msgs
	std_srvs
	trinity_pi    
)

catkin_package()

# Include directories:
include_directories(
    ${catkin_INCLUDE_DIRS}
    src
)

# Executables:
add_executable(joystick_drive
    src/joystick_drive.cpp
)
add_executable(twist_to_motors
    src/twist_to_motors.cpp
)

# Linking:
target_link_libraries(joystick_drive ${catkin_LIBRARIES})
target_link_libraries(twist_to_motors ${catkin_LIBRARIES})
