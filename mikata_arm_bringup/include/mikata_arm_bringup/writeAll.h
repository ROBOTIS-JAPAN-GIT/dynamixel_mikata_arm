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

#ifndef __WRITE_H
#define __WRITE_H

#include "mikata_arm_toolbox/basic_util.h"
#include "mikata_arm_toolbox/kinematics.h"

void writeAll(std::vector<int> id_vec, std::vector<double> q, double time=0) {
  std::vector<int> present_vel = getVelAll(id_vec);
  std::vector<int> overflow;
  std::vector<int> goal_vel = onTime(id_vec, q, time, &overflow);
  clock_t elapsed_time;

  for(int i=0; i<overflow.size(); i++)
    ROS_INFO("ID %d exceeds max velocity", overflow[i]);

  setVelAll(id_vec, goal_vel);
  if (time) elapsed_time = clock();
  dxl_writeAll(id_vec, rad2dxl(q), ADDR_X_GOAL_POSITION, sizeof(int32_t));

  /*
  while(isMoving()) {}
  if (time) {
    elapsed_time = clock() - elapsed_time;
    ROS_INFO("   Goal Time [s]: %f", time);
    ROS_INFO("Elapsed Time [s]: %f", ((double)elapsed_time)/CLOCKS_PER_SEC);
    }
  */

  // Reset velocity to original value
  setVelAll(id_vec, present_vel);
}

#endif
