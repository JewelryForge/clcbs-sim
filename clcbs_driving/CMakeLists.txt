cmake_minimum_required(VERSION 3.0.0)
project(clcbs_driving)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

find_package(yaml-cpp REQUIRED)
find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        message_generation
        std_msgs
        sensor_msgs
        gazebo_msgs
        tf
)
catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs gazebo_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR}
)

add_library(lib src/CarModel.cpp src/FeedbackController.cpp src/StateManager.cpp src/Angle.cpp)
add_executable(clcbs_driving src/main.cpp)
target_link_libraries(clcbs_driving lib ${catkin_LIBRARIES} yaml-cpp)