cmake_minimum_required(VERSION 3.0.2)
project(image_capturing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_capturing_node src/image_capturing_node.cpp)
target_link_libraries(image_capturing_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS image_capturing_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS src/push_button_node.py
 DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)
