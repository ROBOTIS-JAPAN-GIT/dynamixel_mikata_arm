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

#ifndef __CONVERSION_UTIL_H
#define __CONVERSION_UTIL_H

#include <math.h>
#include <vector>

/** Utility for converting Control Table unity values **/

//DYNAMIXEL definitions
#define DXL_MAXIMUM_POSITION_VALUE  4096

//Override for vector
#define OVERRIDE(name, in, out)						\
  std::vector<out> name(std::vector<in> val) {				\
    std::vector<out> res;						\
    for(int i=0; i<val.size(); i++) res.push_back(name(val[i]));	\
    return res; }


//Functions

inline double deg2rad(double val) { return val*M_PI/180.0; }
inline double rad2deg(double val) { return val*180.0/M_PI; }    
inline double dxl2rad(int val) {
  return (double)(val - DXL_MAXIMUM_POSITION_VALUE/2)*2*M_PI/DXL_MAXIMUM_POSITION_VALUE;
}
inline int rad2dxl(double val) {
  if((val < M_PI) && (val >= -M_PI)) return round(val*DXL_MAXIMUM_POSITION_VALUE/(2*M_PI) + DXL_MAXIMUM_POSITION_VALUE/2);
  else return round(val*DXL_MAXIMUM_POSITION_VALUE/(2*M_PI) - DXL_MAXIMUM_POSITION_VALUE/2);
}
inline int deg2dxl(double val) { return round(rad2dxl(deg2rad(val))); }
inline double dxl2deg(int val) { return rad2deg(dxl2rad(val)); }

inline double vel2radsec(int val) { return val*0.229*2*M_PI/60; } //.229
inline int radsec2vel(double val) { return ceil(val*60/(0.229*2*M_PI)); } //round

inline double acel2radsec(int val) { return val*214.577*2*M_PI/3600; }
inline int radsec2acel(double val) { return round(val*3600/(214.577*2*M_PI)); }

inline double cur2torque(int val) { return val*0.00269*1.528734; }


//Overrides

OVERRIDE(deg2rad, double, double)
OVERRIDE(rad2deg, double, double)
OVERRIDE(dxl2rad, int, double)
OVERRIDE(rad2dxl, double, int)
OVERRIDE(deg2dxl, double, int)
OVERRIDE(dxl2deg, int, double)
OVERRIDE(vel2radsec, int, double)
OVERRIDE(radsec2vel, double, int)
OVERRIDE(acel2radsec, int, double)
OVERRIDE(radsec2acel, double, int)
OVERRIDE(cur2torque, int, double)
    
#endif
