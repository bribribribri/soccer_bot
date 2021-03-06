cmake_minimum_required(VERSION 2.8.3)
project(soccer_bot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  dynamic_reconfigure
  OpenCV
)

generate_dynamic_reconfigure_options(
  cfg/Pid.cfg
  cfg/Hue.cfg
)

catkin_package(
  INCLUDE_DIRS cfg
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport dynamic_reconfigure
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(soccer_pid src/soccer_pid.cpp)
add_dependencies(soccer_pid dynamic_reconfigure ${PROJECT_NAME}_gencfg)
target_link_libraries(soccer_pid ${catkin_LIBRARIES})

add_executable(soccer_detector src/soccer_detector.cpp)
target_link_libraries(soccer_detector ${catkin_LIBRARIES})
target_link_libraries(soccer_detector ${OpenCV_LIBRARIES})
