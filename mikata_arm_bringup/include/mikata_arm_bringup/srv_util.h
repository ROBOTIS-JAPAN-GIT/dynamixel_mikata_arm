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

#ifndef __SRV_UTIL_H
#define __SRV_UTIL_H

#include <ros/ros.h>
#include <mikata_arm_toolbox/parameters.h>
#include "mikata_arm_msgs/GetInfo.h"
#include "mikata_arm_msgs/SetInfo.h"
#include "mikata_arm_msgs/onTime.h"
#include "std_srvs/Trigger.h"

/** Utility for calling ROS services defined in setup **/

#define SET_SRV(srv_name, srv_type) ros::ServiceClient client = nh.serviceClient<srv_type>(srv_name); srv_type srv

////
// Function List
////

int srv_read(ros::NodeHandle nh, int id, int addr, int size);
std::vector<int> srv_readAll(ros::NodeHandle nh, std::vector<int> id, int addr, int size);
void srv_write(ros::NodeHandle nh, int id, int val, int addr, int size);
void srv_writeAll(ros::NodeHandle nh, std::vector<int> id, std::vector<int> val, int addr, int size);
void srv_move_onTime(ros::NodeHandle nh, std::vector<int> id, std::vector<double> val, double time=0);
void srv_enableAll(ros::NodeHandle nh);
void srv_disableAll(ros::NodeHandle nh);

////
// Functions
////

int srv_read(ros::NodeHandle nh, int id, int addr, int size) {
  SET_SRV("get_info", mikata_arm_msgs::GetInfo);
  std::vector<int> id_vec;
  id_vec.push_back(id);
  srv.request.id = id_vec;
  srv.request.addr = addr;
  srv.request.size = size;
  client.call(srv);
  return srv.response.data[0];
}

std::vector<int> srv_readAll(ros::NodeHandle nh, std::vector<int> id, int addr, int size)
{
  SET_SRV("get_info", mikata_arm_msgs::GetInfo);
  srv.request.id = id;
  srv.request.addr = addr;
  srv.request.size = size;
  client.call(srv);
  return srv.response.data;
}

void srv_write(ros::NodeHandle nh, int id, int val, int addr, int size) {
  SET_SRV("set_info", mikata_arm_msgs::SetInfo);
  std::vector<int> id_vec;
  id_vec.push_back(id);
  std::vector<int> dxl_val;
  dxl_val.push_back(val);
  srv.request.id = id_vec;
  srv.request.val = dxl_val;
  srv.request.addr = addr;
  srv.request.size = size;
  client.call(srv);
}

void srv_writeAll(ros::NodeHandle nh, std::vector<int> id, std::vector<int> val, int addr, int size) {
  SET_SRV("set_info", mikata_arm_msgs::SetInfo);
  srv.request.id = id;
  srv.request.val = val;
  srv.request.addr = addr;
  srv.request.size = size;
  client.call(srv);
}

void srv_move_onTime(ros::NodeHandle nh, std::vector<int> id, std::vector<double> val, double time) {
  SET_SRV("move_on_time", mikata_arm_msgs::onTime);
  srv.request.id = id;
  srv.request.val_rad = val;
  srv.request.time = time;
  client.call(srv);
}

void srv_enableAll(ros::NodeHandle nh) {
  SET_SRV("enable_all", std_srvs::Trigger);
  client.call(srv);
}

void srv_disableAll(ros::NodeHandle nh) {
  SET_SRV("disable_all", std_srvs::Trigger);
  client.call(srv);
}

#endif
