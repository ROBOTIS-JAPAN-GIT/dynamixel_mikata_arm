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

#include "mikata_arm_bringup/motion_player.h"
#include <cstdlib>

std::string POSE_FILE = std::string(std::getenv("HOME")) + "/.robotis/mikata_arm/positions.txt";
std::string MOTION_FILE = std::string(std::getenv("HOME")) + "/.robotis/mikata_arm/motion.txt";

//msgs
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include "mikata_arm_msgs/dxl_settings.h"
#include "mikata_arm_msgs/dxl_double.h"
#include "mikata_arm_msgs/dxl_int.h"

//srvs
#include "mikata_arm_msgs/GetInfo.h"
#include "mikata_arm_msgs/SetInfo.h"
#include "mikata_arm_msgs/onTime.h"
#include "mikata_arm_msgs/InverseKinematics.h"
#include "std_srvs/Trigger.h"

//call backs
void goal_position_cb (const mikata_arm_msgs::dxl_double::ConstPtr& msg);
void profile_velocity_cb (const mikata_arm_msgs::dxl_double::ConstPtr& msg);
bool onTime_cb(mikata_arm_msgs::onTime::Request &req, mikata_arm_msgs::onTime::Response &res);
bool ik_cb(mikata_arm_msgs::InverseKinematics::Request &req, mikata_arm_msgs::InverseKinematics::Response &res);
bool get_cb(mikata_arm_msgs::GetInfo::Request &req, mikata_arm_msgs::GetInfo::Response &res);
bool set_cb(mikata_arm_msgs::SetInfo::Request &req, mikata_arm_msgs::SetInfo::Response &res);
bool enableAll_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool disableAll_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool motion_on_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool motion_off_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool motion_pause_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool motion_continue_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

MotionPlayer motion_player(POSE_FILE, MOTION_FILE);

int main(int argc, char** argv)
{  
  dxl_setup();
  pingAll();
  setChain();
  
  setVelAll(20);
  setAcelAll(3, false);
  dxl_write(GRIPPER_ID, 30, ADDR_X_PROFILE_VEL, sizeof(int32_t));
  dxl_write(GRIPPER_ID, 250, ADDR_X_P_GAIN, sizeof(int16_t));
  
  ros::init(argc, argv, "bringup_pub");
  ros::NodeHandle n;
  
  //Subscribers
  ros::Subscriber goal_position_sub = n.subscribe("dxl/goal_position", 1000, goal_position_cb);
  ros::Subscriber profile_velocity_sub = n.subscribe("dxl/set_profile_vel", 1000, profile_velocity_cb);

  //Publishers
  ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("dxl/joint_state", 1000);
  ros::Publisher motion_state_pub = n.advertise<std_msgs::String>("motion_player/status", 100);
  ros::Publisher dxl_settings_pub = n.advertise<mikata_arm_msgs::dxl_settings>("dxl/settings", 1000);

  //Services
  ros::ServiceServer get_srv = n.advertiseService("get_info", get_cb);
  ros::ServiceServer set_srv = n.advertiseService("set_info", set_cb);
  ros::ServiceServer onTime_srv = n.advertiseService("move_on_time", onTime_cb);
  ros::ServiceServer ik_srv = n.advertiseService("inverse_kinematics", ik_cb);
  ros::ServiceServer enable_srv = n.advertiseService("enable_all", enableAll_cb);
  ros::ServiceServer disable_srv = n.advertiseService("disable_all", disableAll_cb);
  ros::ServiceServer motion_on_srv = n.advertiseService("motion_player/start", motion_on_cb);
  ros::ServiceServer motion_off_srv = n.advertiseService("motion_player/stop", motion_off_cb);
  ros::ServiceServer motion_pause_srv = n.advertiseService("motion_player/pause", motion_pause_cb);
  ros::ServiceServer motion_continue_srv = n.advertiseService("motion_player/continue", motion_continue_cb);
  
  ROS_INFO("Ready.");
  ros::Rate loop_rate(10);

  while(ros::ok()) {
    //Joint State
    {
      sensor_msgs::JointState msg;
      std::vector<std::string> name = {"joint_1", "joint_2", "joint_3", "joint_4", "gripper_joint_5"};
      std::vector<int> dxl_position = dxl_readAll(ADDR_X_PRESENT_POSITION, sizeof(int32_t));
      std::vector<int> dxl_vel = dxl_readAll(ADDR_X_PRESENT_VELOCITY, sizeof(int32_t));
      std::vector<int> dxl_cur = dxl_readAll(ADDR_X_PRESENT_CURRENT, sizeof(int16_t));
      
      msg.name= name;
      msg.position = dxl2rad(dxl_position);
      msg.velocity = vel2radsec(dxl_vel);
      msg.effort   = cur2torque(dxl_cur);
      msg.header.stamp = ros::Time::now();
    
      joint_state_pub.publish(msg);
    }
    
    //DYNAMIXEL settings
    {
      mikata_arm_msgs::dxl_settings msg;

      std::vector<int> min_pos = dxl_readAll(ADDR_X_MIN_POSITION, sizeof(int32_t));
      std::vector<int> max_pos = dxl_readAll(ADDR_X_MAX_POSITION, sizeof(int32_t));
      std::vector<int> profile_vel = dxl_readAll(ADDR_X_PROFILE_VEL, sizeof(int32_t));
      std::vector<int> p_gain = dxl_readAll(ADDR_X_P_GAIN, sizeof(int32_t));
      std::vector<int> torque_enable = dxl_readAll(ADDR_X_TORQUE_ENABLE, sizeof(int8_t));
      std::vector<int> moving = dxl_readAll(ADDR_X_MOVING, sizeof(int8_t));

      msg.min_pos = dxl2rad(min_pos);
      msg.max_pos = dxl2rad(max_pos);
      msg.profile_vel = vel2radsec(profile_vel);
      msg.p_gain = p_gain;
      msg.torque_enable = torque_enable;
      msg.moving = moving;
      msg.header.stamp = ros::Time::now();

      dxl_settings_pub.publish(msg);
    }

    //Motion Player
    {
      std_msgs::String msg;
      msg.data = motion_player.getStatus();
      motion_state_pub.publish(msg);
      if (motion_player.check()) motion_player.playNext();
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

void goal_position_cb (const mikata_arm_msgs::dxl_double::ConstPtr& msg) {
  writeAll(msg->id, msg->data);
}

void profile_velocity_cb (const mikata_arm_msgs::dxl_double::ConstPtr& msg) {
  setVelAll(msg->id, radsec2vel(msg->data));
}

bool get_cb(mikata_arm_msgs::GetInfo::Request &req, mikata_arm_msgs::GetInfo::Response &res) {
  std::vector<int> dxl_data = dxl_readAll(req.addr, req.size);
  for(int i=0; i<req.id.size(); i++) 
    res.data.push_back(dxl_data[req.id[i]-1]);
  return true;
}

bool set_cb(mikata_arm_msgs::SetInfo::Request &req, mikata_arm_msgs::SetInfo::Response &res) {
  dxl_writeAll(req.id, req.val, req.addr, req.size);
  std::stringstream ss;
  ss << "Wrote [";
  for(int i=0; i<req.val.size(); i++) {
    ss << req.val[i];
    if(i!=(req.val.size()-1)) ss << ", ";
  }
  ss << "] to " << req.addr;
  res.success = true;
  res.message = ss.str();
  return true;
}

bool onTime_cb(mikata_arm_msgs::onTime::Request &req, mikata_arm_msgs::onTime::Response &res) {
  writeAll(req.id, req.val_rad, req.time);
  std::stringstream ss;
  ss << "Wrote [";
  for(int i=0; i<req.val_rad.size(); i++) {
    ss << req.val_rad[i];
    if(i!=(req.val_rad.size()-1)) ss << ", ";
  }
  ss << "] to " << ADDR_X_GOAL_POSITION;
  res.success = true;
  res.message = ss.str();
  return true;
}

bool ik_cb(mikata_arm_msgs::InverseKinematics::Request &req, mikata_arm_msgs::InverseKinematics::Response &res) {
  std::vector<int> id_vec = {1,2,3,4};
  std::vector<double> cur_q = readAll();     // Present Joint State
  std::vector<double> q;                     // Goal Joint State
  Vector3d goal;                             // Goal End Effector Position

  q.resize(LINK_NUM);

  for(int i=0; i<LINK_NUM; i++)
    chain[i].setAngle(cur_q[i]);

  goal[0] = 1000 * req.x;
  goal[1] = 1000 * req.y;
  goal[2] = 1000 * req.z;

  if (solveIK(goal)) {
    for(int i=0; i<LINK_NUM; i++) q[i]=chain[i].getAngle();
    writeAll(id_vec, q, req.time);
    res.success = true;
  }
  else res.success = false;
    
  res.val_rad = q;
  return true;
}

#define MAKE_CB(name, func, msg) bool name(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) { func; res.message = msg; res.success = true; return true; }

MAKE_CB(enableAll_cb, enableAll(), "Enabled all")
MAKE_CB(disableAll_cb, disableAll(), "Disabled all")
MAKE_CB(motion_on_cb, enableAll(); motion_player.start(), "MotionPlayer started")
MAKE_CB(motion_off_cb, disableAll(); motion_player.stop(), "MotionPlayer disabled")
MAKE_CB(motion_pause_cb, motion_player.pause(), "MotionPlayer paused")
MAKE_CB(motion_continue_cb, motion_player.unpause(), "MotionPlayer continued")
