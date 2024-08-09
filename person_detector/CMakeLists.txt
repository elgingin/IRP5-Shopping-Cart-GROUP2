cmake_minimum_required(VERSION 3.0.2)
project(person_detector)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
geometry_msgs
darknet_ros_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(

CATKIN_DEPENDS roscpp std_msgs geometry_msgs darknet_ros_msgs
)

include_directories(
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

add_executable(person_detector_node src/person_detector_node.cpp)

target_link_libraries(person_detector_node
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)

catkin_install_python(PROGRAMS src/geometrytoackerman.py
 DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION)
)
