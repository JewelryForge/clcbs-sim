cmake_minimum_required(VERSION 3.0.0)
project(clcbs_driving)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        message_generation
        std_msgs
        sensor_msgs
        gazebo_msgs
        )

catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs gazebo_msgs
)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(lib src/CarModel.cpp include/CarModel.h)
add_executable(clcbs_driving src/main.cpp src/CarModel.cpp include/CarModel.h src/Angle.cpp include/Angle.h include/PID.hpp)
target_link_libraries(clcbs_driving lib ${catkin_LIBRARIES})