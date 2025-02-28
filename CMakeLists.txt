cmake_minimum_required(VERSION 2.8.3)
project(find_cup_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# add_definitions(-Wall -O3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -funsafe-loop-optimizations -fsee -funroll-loops -fno-math-errno -funsafe-math-optimizations -ffinite-math-only -fno-signed-zeros ")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  cv_bridge
  tf
  cmake_modules
  nodelet
  pluginlib
  geometry_msgs
  image_transport
  image_geometry
  apriltag_ros
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED) 
find_package(PkgConfig)
# include_directories(${find_cup_ros_INCLUDE_DIRS})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   CupInfo.msg
   CupDetection.msg
   CupDetectionArray.msg
   
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs image_transport roscpp sensor_msgs std_msgs message_runtime cv_bridge tf nodelet pluginlib apriltag_ros
  DEPENDS OpenCV   
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}  
  include
)
 
include_directories(${apriltag_INCLUDE_DIRS}/apriltag)
link_directories(${apriltag_LIBDIR})
 
add_executable(${PROJECT_NAME}_continuous_node src/cup_find_ros_continuous_node.cpp  src/fast_find_ellipse_detector.cpp src/common.cpp src/EllipseDetectorYaed.cpp)
add_dependencies(${PROJECT_NAME}_continuous_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_continuous_node  ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_general_node src/cup_find_ros_general_node.cpp  src/fast_find_ellipse_detector.cpp src/common.cpp src/EllipseDetectorYaed.cpp)
add_dependencies(${PROJECT_NAME}_general_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_general_node  ${catkin_LIBRARIES})

add_executable(cup_find_and_measure_node src/cup_find_and_measure_node.cpp  src/fast_find_ellipse_detector.cpp src/common.cpp src/EllipseDetectorYaed.cpp)
add_dependencies(cup_find_and_measure_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(cup_find_and_measure_node  ${catkin_LIBRARIES})
 
add_executable(${PROJECT_NAME}_stereo_detection_node src/stereo_detection.cpp  src/fast_find_ellipse_detector.cpp src/common.cpp src/EllipseDetectorYaed.cpp)
add_dependencies(${PROJECT_NAME}_stereo_detection_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_stereo_detection_node  ${catkin_LIBRARIES})

add_executable(find_multi_cup_node src/find_multi_cup_node.cpp  src/fast_find_ellipse_detector.cpp src/common.cpp src/EllipseDetectorYaed.cpp)
add_dependencies(find_multi_cup_node ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(find_multi_cup_node  ${catkin_LIBRARIES})


#############
## Install ##
#############

 

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 

install(TARGETS  
  ${PROJECT_NAME}_continuous_node  
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
 
