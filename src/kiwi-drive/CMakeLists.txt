cmake_minimum_required(VERSION 2.8.3)
project(kiwi-drive)

add_compile_options(
    -std=c++14
    -Wall
    -Wextra
    -Wfatal-errors
)

find_package(catkin REQUIRED COMPONENTS roscpp)

# Include directories:
include_directories(
    src
)

# Executables:
add_executable(joystick_drive
    src/joystick_drive.cpp
)

# Linking:
target_link_libraries(joystick_drive ${catkin_LIBRARIES})

#set(gpio_LIBS "-lpthread -lpigpiod_if2")
