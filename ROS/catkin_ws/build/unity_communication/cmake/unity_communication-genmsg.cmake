# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "unity_communication: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iunity_communication:/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(unity_communication_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg" NAME_WE)
add_custom_target(_unity_communication_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "unity_communication" "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg" "geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(unity_communication
  "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/unity_communication
)

### Generating Services

### Generating Module File
_generate_module_cpp(unity_communication
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/unity_communication
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(unity_communication_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(unity_communication_generate_messages unity_communication_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg" NAME_WE)
add_dependencies(unity_communication_generate_messages_cpp _unity_communication_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(unity_communication_gencpp)
add_dependencies(unity_communication_gencpp unity_communication_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS unity_communication_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(unity_communication
  "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/unity_communication
)

### Generating Services

### Generating Module File
_generate_module_eus(unity_communication
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/unity_communication
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(unity_communication_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(unity_communication_generate_messages unity_communication_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg" NAME_WE)
add_dependencies(unity_communication_generate_messages_eus _unity_communication_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(unity_communication_geneus)
add_dependencies(unity_communication_geneus unity_communication_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS unity_communication_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(unity_communication
  "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/unity_communication
)

### Generating Services

### Generating Module File
_generate_module_lisp(unity_communication
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/unity_communication
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(unity_communication_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(unity_communication_generate_messages unity_communication_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg" NAME_WE)
add_dependencies(unity_communication_generate_messages_lisp _unity_communication_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(unity_communication_genlisp)
add_dependencies(unity_communication_genlisp unity_communication_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS unity_communication_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(unity_communication
  "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/unity_communication
)

### Generating Services

### Generating Module File
_generate_module_nodejs(unity_communication
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/unity_communication
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(unity_communication_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(unity_communication_generate_messages unity_communication_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg" NAME_WE)
add_dependencies(unity_communication_generate_messages_nodejs _unity_communication_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(unity_communication_gennodejs)
add_dependencies(unity_communication_gennodejs unity_communication_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS unity_communication_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(unity_communication
  "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/unity_communication
)

### Generating Services

### Generating Module File
_generate_module_py(unity_communication
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/unity_communication
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(unity_communication_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(unity_communication_generate_messages unity_communication_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/matt/Waterloo/FYDP/ROS/catkin_ws/src/unity_communication/msg/carPosition.msg" NAME_WE)
add_dependencies(unity_communication_generate_messages_py _unity_communication_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(unity_communication_genpy)
add_dependencies(unity_communication_genpy unity_communication_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS unity_communication_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/unity_communication)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/unity_communication
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(unity_communication_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(unity_communication_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/unity_communication)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/unity_communication
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(unity_communication_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(unity_communication_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/unity_communication)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/unity_communication
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(unity_communication_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(unity_communication_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/unity_communication)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/unity_communication
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(unity_communication_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(unity_communication_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/unity_communication)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/unity_communication\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/unity_communication
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(unity_communication_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(unity_communication_generate_messages_py geometry_msgs_generate_messages_py)
endif()
