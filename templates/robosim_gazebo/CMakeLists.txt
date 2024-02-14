cmake_minimum_required(VERSION 3.0.2)
project(@(config.robot_name)_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS)