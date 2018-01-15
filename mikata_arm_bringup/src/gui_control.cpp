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

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <mikata_arm_toolbox/dxl_ctrl_table.h>
#include <mikata_arm_toolbox/conversion_util.h>
#include "mikata_arm_bringup/srv_util.h"
#include "mikata_arm_bringup/PositionConfig.h"
#include "mikata_arm_bringup/VelocityConfig.h"

/** simple dynamic reconfigure GUI for mikata_arm_bringup **/

std::vector<int> id_vec = {1,2,3,4,5};

class GUIControl
{
public:
  GUIControl();
  void position_cb(mikata_arm_bringup::PositionConfig &config, uint32_t level);
  void velocity_cb(mikata_arm_bringup::VelocityConfig &config, uint32_t level);
  
private:
  ros::NodeHandle nh_a;
  ros::NodeHandle nh_b;
  ros::NodeHandle srv_nh;
  dynamic_reconfigure::Server<mikata_arm_bringup::PositionConfig> server_a;
  dynamic_reconfigure::Server<mikata_arm_bringup::VelocityConfig> server_b;
  dynamic_reconfigure::Server<mikata_arm_bringup::PositionConfig>::CallbackType f_a;
  dynamic_reconfigure::Server<mikata_arm_bringup::VelocityConfig>::CallbackType f_b;

};

GUIControl::GUIControl()
{
  new (&nh_a) ros::NodeHandle("~/Position");
  new (&nh_b) ros::NodeHandle("~/Velocity");
  new (&server_a) dynamic_reconfigure::Server<mikata_arm_bringup::PositionConfig>(nh_a);
  new (&server_b) dynamic_reconfigure::Server<mikata_arm_bringup::VelocityConfig>(nh_b);

  f_a = boost::bind(&GUIControl::position_cb, this, _1, _2);
  f_b = boost::bind(&GUIControl::velocity_cb, this, _1, _2);

  server_a.setCallback(f_a);
  server_b.setCallback(f_b);
    
}

void GUIControl::position_cb(mikata_arm_bringup::PositionConfig &config, uint32_t level) {
  if(config.TORQUE_ENABLE) srv_enableAll(srv_nh);
  if(!config.TORQUE_ENABLE) srv_disableAll(srv_nh);
    
  std::vector<double> rad_val = {config.ID_1, config.ID_2, config.ID_3, config.ID_4, config.ID_5};
  srv_move_onTime(srv_nh, id_vec, rad_val);
}

void GUIControl::velocity_cb(mikata_arm_bringup::VelocityConfig &config, uint32_t level) {
  std::vector<int> dxl_val = {config.ID_1, config.ID_2, config.ID_3, config.ID_4, config.ID_5};
  srv_writeAll(srv_nh, id_vec, dxl_val, ADDR_X_PROFILE_VEL, 4);
}
  

int main(int argc, char** argv) {
  ros::init(argc, argv, "gui_control");

  GUIControl ctrl;
  
  ros::spin();

  return 0;
}
