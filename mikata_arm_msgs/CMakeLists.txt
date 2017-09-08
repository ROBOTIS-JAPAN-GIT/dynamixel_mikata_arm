cmake_minimum_required(VERSION 2.8.3)
project(mikata_arm_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  dxl_int.msg
  dxl_double.msg
  dxl_settings.msg
)

add_service_files(
  FILES
  GetInfo.srv
  SetInfo.srv
  onTime.srv
  InverseKinematics.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
