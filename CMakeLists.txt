cmake_minimum_required(VERSION 3.5)
project(quadruped_robot)

set(CMAKE_CXX_STANDARD 14)
set(BUILD_TYPE_RELEASE TRUE)

find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/third-party)
add_subdirectory(src/robot)
add_subdirectory(src/common)
add_subdirectory(src/user/MIT_Controller)

include_directories("src/user/MIT_Controller")
include_directories("src/user/MIT_Controller/Controllers")
include_directories("src/robot/include")
include_directories("src/common/include/")
include_directories("src/common/FootstepPlanner")
include_directories("src/third-party/")
include_directories(SYSTEM "src/third-party/qpOASES/include")
include_directories("src/third-party/ParamHandler")
include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

add_executable(quadruped_ctrl src/main.cpp)
target_link_libraries(quadruped_ctrl ${catkin_LIBRARIES})
target_link_libraries(quadruped_ctrl ctrller)
target_link_libraries(quadruped_ctrl robot biomimetics)
target_link_libraries(quadruped_ctrl dynacore_param_handler qpOASES)
target_link_libraries(quadruped_ctrl Goldfarb_Optimizer osqp)
target_link_libraries(quadruped_ctrl WBC_Ctrl)
target_link_libraries(quadruped_ctrl VisionMPC)
add_dependencies(quadruped_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


