################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(bartending_server)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  bartending_gripper_service
  open_manipulator_msgs
  dynamixel_workbench_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_service_files(
  FILES
  BartenderCocktailRequest.srv
)

generate_messages(
  DEPENDENCIES
)

################################################################################
## Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp message_runtime
)

################################################################################
# Build
################################################################################

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bartending_server_node src/bartending_server.cpp)
add_dependencies(bartending_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bartending_server_node ${catkin_LIBRARIES})

#################################################################################
## Install
#################################################################################
install(TARGETS bartending_server_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
