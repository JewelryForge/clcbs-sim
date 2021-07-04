cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 14)

project(clcbs_driving)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OsqpEigen REQUIRED)
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
        hunter_msgs
)
catkin_package(
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime sensor_msgs gazebo_msgs tf hunter_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
)

add_library(lib src/CarModel.cpp src/LocalPlanner.cpp src/StateManager.cpp src/Angle.cpp src/PlanVisualizer.cpp src/WheelOdometer.cpp)

add_executable(clcbs_driving src/main.cpp)

target_link_libraries(clcbs_driving lib OsqpEigen::OsqpEigen ${catkin_LIBRARIES} yaml-cpp ${Boost_LIBRARIES})