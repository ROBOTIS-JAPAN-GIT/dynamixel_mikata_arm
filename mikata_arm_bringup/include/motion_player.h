#include <ros/ros.h>
#include <iomanip>
#include "writeAll.h"
#include "../../mikata_arm_toolbox/include/teach.h"

struct motion{
  node pose;
  double time;
};

template <class T> std::ostream& operator << (std::ostream& os, const std::vector<T>& v) {
  os << "[ ";
  for (typename std::vector<T>::const_iterator it = v.begin(); it != v.end(); ++it)
    os << *it << " ";
  os << "]";
  return os;
}

class MotionPlayer {

private:
  Teach positions;
  std::vector<motion> motions;
  ros::Duration motion_duration_time;
  ros::Time motion_start_time;
  std::string status;
  bool is_enabled;
  bool is_paused;
  int n;

public:
  MotionPlayer (std::string pose_file, std::string motion_file);
  std::string getStatus () { return status; }
  void playNext ();
  bool check ();
  void start ();
  void stop ();
  void pause ();
  void unpause ();
};

MotionPlayer::MotionPlayer (std::string pose_file, std::string motion_file) {
  read_file(pose_file, [&] (std::string name, std::vector<int> id_vec, std::vector<double> q)
	    { positions.addNode(name, id_vec, q); } );
  read_file(motion_file, [&] (std::string name, std::vector<int> n, std::vector<double> time)
	    { motion m;
	      node* pose = positions.getNode(name);
	      if (pose) m.pose = *pose;
	      else {
		if (name[0] != '&') {
		  std::stringstream ss;
		  ss << "Could not find " << name << " in positions.txt.";
		  throw std::runtime_error(ss.str());
		}
		else m.pose.name = name;
	      }
	      if (time.size()) m.time = time[0];
	      else m.time = 0.0;
	      motions.push_back(m);
	    } );

  motion_duration_time = ros::Duration(0.0);
  status = "STOPPED";
  is_enabled = false;
  is_paused = false;
  n = 0;

  ROS_INFO("Successfully loaded %d lines from %s", (int)positions.getSize(), pose_file.c_str());
  ROS_INFO("Successfully loaded %d lines from %s", (int)motions.size(), motion_file.c_str());
  ROS_INFO("MotionPlayer init succeeded.");
}

bool MotionPlayer::check () {
  if (is_enabled && !is_paused) {
    ros::Duration motion_elapsed_time = ros::Time::now() - motion_start_time;
    if (motion_elapsed_time > motion_duration_time) return true;
  }
  return false;
}

void MotionPlayer::playNext () {
  if (n == motions.size()) {
    stop();
    ROS_INFO("MotionPlayer Finished.");
    return;
  }
  motion motion = motions[n++];
  if (motion.pose.name == "&PAUSE") { is_paused = true; return; }
  if (motion.pose.name == "&LOOP") { n = 0; return; }
  if (motion.pose.name == "&TORQUE_ENABLE") { enableAll(); return; }
  if (motion.pose.name == "&TORQUE_DISABLE") { disableAll(); return; }
    
  motion_start_time = ros::Time::now();
  motion_duration_time = ros::Duration(motion.time);
  if (motion.pose.name[0] != '&') writeAll(motion.pose.id_vec, motion.pose.q, motion.time);
  
  std::cout << std::endl << "start motion: " << motion.pose.name;
  std::cout << " , time: " << std::fixed << std::setprecision(2) << motion.time << std::endl;
  std::cout << "  ID vector:    "  << motion.pose.id_vec << std::endl;
  std::cout << "  Angle vector: " << std::defaultfloat << motion.pose.q << std::endl;
}
  
void MotionPlayer::start () {
  is_enabled = true;
  status = "RUNNING";
  motion_start_time = ros::Time::now();
}

void MotionPlayer::stop () {
  is_enabled = false;
  is_paused = false;
  status = "STOPPED";
  n = 0;
  ROS_INFO ("MotionPlayer disabled");
}

void MotionPlayer::pause () {
  is_paused = true;
  status = "PAUSED";
}

void MotionPlayer::unpause () {
  is_paused = false;
  status = "RUNNING";
}
