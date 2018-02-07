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

#ifndef __DXL_CTRL_TABLE_H
#define __DXL_CTRL_TABLE_H

/** Control Table Addresses for DYNAMIXEL XM430-W350 and XM430-V350 **/

#define ADDR_X_MODEL_NUMBER         0
#define ADDR_X_VERSION_OF_FIRMWARE  6
#define ADDR_X_DRIVE_MODE           10
#define ADDR_X_OPERATING_MODE       11
#define ADDR_X_HOMING_OFFSET        20
#define ADDR_X_MOVING_THRESHOLD     24
#define ADDR_X_ACEL_LIM             40
#define ADDR_X_VELOCITY_LIM         44
#define ADDR_X_MAX_POSITION         48
#define ADDR_X_MIN_POSITION         52
#define ADDR_X_TORQUE_ENABLE        64
#define ADDR_X_P_GAIN               84
#define ADDR_X_GOAL_CURRENT         102
#define ADDR_X_GOAL_VELOCITY        104
#define ADDR_X_PROFILE_ACEL         108
#define ADDR_X_PROFILE_VEL          112
#define ADDR_X_GOAL_POSITION        116
#define ADDR_X_MOVING               122
#define ADDR_X_PRESENT_CURRENT      126
#define ADDR_X_PRESENT_VELOCITY     128
#define ADDR_X_PRESENT_POSITION     132

#endif
