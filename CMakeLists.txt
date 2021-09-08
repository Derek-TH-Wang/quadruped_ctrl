cmake_minimum_required(VERSION 3.0.2)
project(quadruped_ctrl)

set(CMAKE_CXX_FLAGS "-std=c++14 -Werror -Wall -Wextra" CACHE STRING "" FORCE)
set(CMAKE_BUILD_TYPE "Release")

find_package (Eigen3 REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
    commandDes.msg
)

add_service_files(
  FILES
    QuadrupedCmd.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    std_msgs
    message_runtime
    geometry_msgs
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS}
  include
  include/${PROJECT_NAME}
  ${Eigen3_INCLUDE_DIRS}
  src
)

add_subdirectory(src/qpOASES)
add_subdirectory(src/JCQP)
add_subdirectory(src/osqp)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/MPC_Ctrl/ConvexMPCLocomotion.cpp
  src/MPC_Ctrl/Gait.cpp
  src/MPC_Ctrl/SparseCMPC.cpp
  src/MPC_Ctrl/SparseCMPC_Math.cpp
  src/MPC_Ctrl/OsqpTriples.cpp
  src/MPC_Ctrl/convexMPC_interface.cpp
  src/MPC_Ctrl/SolverMPC.cpp
  src/MPC_Ctrl/RobotState.cpp
  src/Controllers/FootSwingTrajectory.cpp
  src/Controllers/LegController.cpp
  src/Controllers/DesiredStateCommand.cpp
  src/Controllers/OrientationEstimator.cpp
  src/Controllers/PositionVelocityEstimator.cpp
  src/Controllers/SafetyChecker.cpp
  src/Dynamics/Quadruped.cpp
  src/Dynamics/FloatingBaseModel.cpp
  src/GaitCtrller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  JCQP
  qpOASES
  osqp
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  scripts/walking_simulation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    config
    models
    msg
    rviz
    urdf
    worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
