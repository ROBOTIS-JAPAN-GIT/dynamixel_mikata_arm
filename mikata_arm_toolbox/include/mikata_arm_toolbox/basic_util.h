#ifndef __BASIC_UTIL_H
#define __BASIC_UTIL_H

#include <unistd.h>
#include "dxl_util.h"
#include "dxl_ctrl_table.h"
#include "conversion_util.h"

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
inline void setaAcelAll(int val, bool gripper=true);
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
    
#endif
