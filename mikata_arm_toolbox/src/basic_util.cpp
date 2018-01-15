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

#include <unistd.h>
#include "mikata_arm_toolbox/basic_util.h"

/** Utility for controlling position, velocity and acceleration of Mikata Arm 4DOF **/

////
// Functions
////

std::vector<int> onTime(std::vector<int> id_vec, std::vector<double> q, double time, std::vector<int>* overflow)        // Move to joint state 'q' in time 'time'
// default: time=0, overflow=NULL;
{
  if(q.size() != id_vec.size())
    throw std::runtime_error("writeAll: Sizes do not match");
     
  std::vector<double> present_position = readAll(id_vec);
  std::vector<int> present_vel = getVelAll(id_vec);
  std::vector<double> acel = acel2radsec(dxl_readAll(id_vec, ADDR_X_PROFILE_ACEL, sizeof(int32_t)));
  std::vector<int> max_vel = dxl_readAll(id_vec, ADDR_X_VELOCITY_LIM, sizeof(int32_t));
  std::vector<double> diff;
  std::vector<int> goal_vel;
  std::vector<int> error;

  // Get Joint state and velocity  
  for(int i=0; i<id_vec.size(); i++) {
    diff.push_back(fabs(present_position[i] - q[i]));
    if(!present_vel[i]) present_vel[i] = dxl_read(id_vec[i], ADDR_X_VELOCITY_LIM, sizeof(int32_t));
    if(!acel[i]) acel[i] = dxl_read(id_vec[i], ADDR_X_ACEL_LIM, sizeof(int32_t));
  }

  if(time) {
    // Calculate velocity for given time
    for (int i=0; i<id_vec.size(); i++) {
      int val;
      double delta = pow(time*acel[i],2) - (4*acel[i]*diff[i]);
      if (delta > 0)
	val = radsec2vel((time*acel[i] - sqrt(delta))/2.0);
      else
	val = radsec2vel(2*diff[i]/time);

      if (val > max_vel[i]) {
	val = max_vel[i];
	error.push_back(id_vec[i]);
      }

      goal_vel.push_back(val);
    }
  }

  // If time=0 adjust according to profile velocity
  if(!time) {
    // Get max time
    double max_time=0;
    for(int i=0; i<id_vec.size(); i++) {
      double t = diff[i] / vel2radsec(present_vel[i]);
      if (t > max_time) max_time=t;
    }
    // Calculate velocity for max time
    for(int i=0; i<id_vec.size(); i++)
      goal_vel.push_back(radsec2vel(diff[i]/max_time));
  }

  *overflow = error;
  return goal_vel;
}

std::vector<int> onTime(std::vector<double> q, double time, std::vector<int>* overflow, bool gripper)        // Move to joint state 'q' in time 'time'
// default: time=0, overflow=NULL, gripper=false
{
  std::vector<int> id_vec;
  for(int id=1; id<=LINK_NUM+gripper; id++)
    id_vec.push_back(id);
  return onTime(id_vec, q, time, overflow);
}

void getRange(int id, double* min_val, double* max_val)        // Get max and min position limit
{
  int max_pos = dxl_read(id, ADDR_X_MAX_POSITION, sizeof(int32_t));
  int min_pos = dxl_read(id, ADDR_X_MIN_POSITION, sizeof(int32_t));
  int offset  = dxl_read(id, ADDR_X_HOMING_OFFSET, sizeof(int32_t));
  int sign    = dxl_read(id, ADDR_X_DRIVE_MODE, sizeof(int8_t));
  if(sign)
    min_pos -= offset;     //max_pos -= offset;
  if(!sign)
    min_pos += offset;     //max_pos += offset;
  
  *min_val = dxl2deg(min_pos);
  *max_val = dxl2deg(max_pos);
}

bool isMoving()        // Detect motion
{
  std::vector<int> val = dxl_readAll(ADDR_X_MOVING, sizeof(int8_t));
  std::cout << "***MOVING***" << std::endl;
  for(int i=0; i<val.size(); i++)
    if(val[i]) return true;
  usleep(100000);
  return false;
}

