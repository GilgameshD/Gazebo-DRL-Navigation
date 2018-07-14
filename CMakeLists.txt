cmake_minimum_required(VERSION 2.8.3)
project(gazebo_drl_env)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    gazebo_msgs
    cv_bridge
    #actor_services
    image_transport
    tf
    message_generation
    visualization_msgs
)

find_package(OpenCV REQUIRED)
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Generate services in the 'srv' folder
add_service_files(
    FILES
    PytorchRL.srv
    SocialForce.srv
    SimpleCtrl.srv
)

add_message_files(
  FILES
  control_msgs.msg
  state_msgs.msg
  control_group_msgs.msg
  state_group_msgs.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs    # Or other packages containing msgs
  geometry_msgs
  sensor_msgs
)

catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

include_directories(./include)
add_executable(task_env_service src/main.cpp src/task_env_io.cpp src/position_generator.cpp)
target_link_libraries(task_env_service ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${GAZEBO_LIBRARIES})
