/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef __BASIC_UTIL_H
#define __BASIC_UTIL_H

#include <unistd.h>
#include "mikata_arm_toolbox/dxl_util.h"
#include "mikata_arm_toolbox/dxl_ctrl_table.h"
#include "mikata_arm_toolbox/conversion_util.h"

/** Utility for controlling position, velocity and acceleration of Mikata Arm 4DOF **/

////
// Function List
////

inline std::vector<double> readAll(std::vector<int> id_vec);
inline std::vector<double> readAll(bool gripper=false);
inline std::vector<double> readAll_deg(bool gripper=false);
inline std::vector<int> getVelAll(std::vector<int> id_vec);
inline std::vector<int> getVelAll(bool gripper=true);
inline void setVelAll(std::vector<int> id_vec, std::vector<int> val);
inline void setVelAll(int val, bool gripper=true);
inline void setAcelAll(int val, bool gripper=true);
inline void enableAll(bool gripper=true);
inline void disableAll(bool gripper=true);
void write(int id, double rad);
void write_deg(int id, double deg);
std::vector<int> onTime(std::vector<int> id_vec, std::vector<double> q, double time=0, std::vector<int>* overflow=NULL);
std::vector<int> onTime(std::vector<double> q,  double time=0, std::vector<int>* overflow=NULL, bool gripper=false);
void getRange(int id, double* min_val, double* max_val);
bool isMoving();

////
// Functions
////

inline std::vector<double> readAll(std::vector<int> id_vec) {
  return dxl2rad(dxl_readAll(id_vec, ADDR_X_PRESENT_POSITION, sizeof(int32_t)));
}
inline std::vector<double> readAll(bool gripper) {
  return dxl2rad(dxl_readAll(ADDR_X_PRESENT_POSITION, sizeof(int32_t), gripper));
}
inline std::vector<double> readAll_deg(bool gripper) {
  return dxl2deg(dxl_readAll(ADDR_X_PRESENT_POSITION, sizeof(int32_t), gripper));
}
inline std::vector<int> getVelAll(std::vector<int> id_vec) {
  return dxl_readAll(id_vec, ADDR_X_PROFILE_VEL, sizeof(int32_t));
}
inline std::vector<int> getVelAll(bool gripper) {
  return dxl_readAll(ADDR_X_PROFILE_VEL, sizeof(int32_t), gripper);
}
inline void setVelAll(std::vector<int> id_vec, std::vector<int> val) {
  dxl_writeAll(id_vec, val, ADDR_X_PROFILE_VEL, sizeof(int32_t));
}
inline void setVelAll(int val, bool gripper) { 
  dxl_writeAll(val, ADDR_X_PROFILE_VEL, sizeof(int32_t), gripper);
}
inline void setAcelAll(int val, bool gripper) { 
  dxl_writeAll(val, ADDR_X_PROFILE_ACEL, sizeof(int32_t), gripper);
}
inline void enableAll(bool gripper) {
  dxl_writeAll(1, ADDR_X_TORQUE_ENABLE, sizeof(int8_t), gripper);
}
inline void disableAll(bool gripper) {
  dxl_writeAll(0, ADDR_X_TORQUE_ENABLE, sizeof(int8_t), gripper);
}
inline void write(int id, double rad) {
  dxl_write(id, rad2dxl(rad), ADDR_X_GOAL_POSITION, sizeof(int32_t));
  while(isMoving()) {}
}
inline void write_deg(int id, double deg) {
  dxl_write(id, deg2dxl(deg), ADDR_X_GOAL_POSITION, sizeof(int32_t));
  while(isMoving()) {}
}

#endif
