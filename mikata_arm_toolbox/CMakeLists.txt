cmake_minimum_required(VERSION 2.8.3)
project(mikata_arm_toolbox)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamixel_sdk
  cmake_modules
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mikata_arm_toolbox
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(mikata_arm_toolbox src/basic_util.cpp src/dxl_util.cpp src/kinematics.cpp)

add_executable(dxl_factory_reset src/dxl_factory_reset.cpp)
add_executable(dxl_setup src/dxl_setup.cpp)
add_executable(cli_control src/cli_control.cpp)

target_link_libraries(mikata_arm_toolbox ${catkin_LIBRARIES})
target_link_libraries(dxl_factory_reset ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(dxl_setup ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(cli_control ${catkin_LIBRARIES} ${PROJECT_NAME})
