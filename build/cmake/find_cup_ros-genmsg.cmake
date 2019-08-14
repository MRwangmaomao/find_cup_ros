# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "find_cup_ros: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ifind_cup_ros:/home/wpr/catkin_ws/src/find_cup_ros/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(find_cup_ros_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg" NAME_WE)
add_custom_target(_find_cup_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "find_cup_ros" "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Point"
)

get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg" NAME_WE)
add_custom_target(_find_cup_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "find_cup_ros" "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:find_cup_ros/CupDetection:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg" NAME_WE)
add_custom_target(_find_cup_ros_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "find_cup_ros" "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_cup_ros
)
_generate_msg_cpp(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_cup_ros
)
_generate_msg_cpp(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_cup_ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(find_cup_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_cup_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(find_cup_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(find_cup_ros_generate_messages find_cup_ros_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_cpp _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_cpp _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_cpp _find_cup_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(find_cup_ros_gencpp)
add_dependencies(find_cup_ros_gencpp find_cup_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS find_cup_ros_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/find_cup_ros
)
_generate_msg_eus(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/find_cup_ros
)
_generate_msg_eus(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/find_cup_ros
)

### Generating Services

### Generating Module File
_generate_module_eus(find_cup_ros
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/find_cup_ros
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(find_cup_ros_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(find_cup_ros_generate_messages find_cup_ros_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_eus _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_eus _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_eus _find_cup_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(find_cup_ros_geneus)
add_dependencies(find_cup_ros_geneus find_cup_ros_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS find_cup_ros_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_cup_ros
)
_generate_msg_lisp(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_cup_ros
)
_generate_msg_lisp(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_cup_ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(find_cup_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_cup_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(find_cup_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(find_cup_ros_generate_messages find_cup_ros_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_lisp _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_lisp _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_lisp _find_cup_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(find_cup_ros_genlisp)
add_dependencies(find_cup_ros_genlisp find_cup_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS find_cup_ros_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/find_cup_ros
)
_generate_msg_nodejs(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/find_cup_ros
)
_generate_msg_nodejs(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/find_cup_ros
)

### Generating Services

### Generating Module File
_generate_module_nodejs(find_cup_ros
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/find_cup_ros
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(find_cup_ros_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(find_cup_ros_generate_messages find_cup_ros_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_nodejs _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_nodejs _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_nodejs _find_cup_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(find_cup_ros_gennodejs)
add_dependencies(find_cup_ros_gennodejs find_cup_ros_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS find_cup_ros_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_cup_ros
)
_generate_msg_py(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_cup_ros
)
_generate_msg_py(find_cup_ros
  "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_cup_ros
)

### Generating Services

### Generating Module File
_generate_module_py(find_cup_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_cup_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(find_cup_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(find_cup_ros_generate_messages find_cup_ros_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetection.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_py _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupDetectionArray.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_py _find_cup_ros_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/wpr/catkin_ws/src/find_cup_ros/msg/CupInfo.msg" NAME_WE)
add_dependencies(find_cup_ros_generate_messages_py _find_cup_ros_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(find_cup_ros_genpy)
add_dependencies(find_cup_ros_genpy find_cup_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS find_cup_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_cup_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/find_cup_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(find_cup_ros_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(find_cup_ros_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(find_cup_ros_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/find_cup_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/find_cup_ros
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(find_cup_ros_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(find_cup_ros_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(find_cup_ros_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_cup_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/find_cup_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(find_cup_ros_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(find_cup_ros_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(find_cup_ros_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/find_cup_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/find_cup_ros
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(find_cup_ros_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(find_cup_ros_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(find_cup_ros_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_cup_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_cup_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/find_cup_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(find_cup_ros_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(find_cup_ros_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(find_cup_ros_generate_messages_py sensor_msgs_generate_messages_py)
endif()
