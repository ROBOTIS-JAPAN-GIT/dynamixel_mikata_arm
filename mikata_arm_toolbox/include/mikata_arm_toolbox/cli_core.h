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

#ifndef __CORE_H
#define __CORE_H

#include "kinematics.h"
#include "teach.h"
#include "basic_util.h"
#include <type_traits>
#include <termios.h>
#include <fcntl.h>
#include <cstdlib>
#include <time.h>

/** Utility used in cli_control **/


std::string FILENAME = std::string(std::getenv("HOME")) + "/.robotis/mikata_arm/positions.txt";

////
// Key Binding
////

#define ESC    27
#define YES    121
#define YES_C  89
#define NO     110
#define NO_C   78


////
// Functions List
////

//in-out util
void clear();
int getch();
void wait_key();
bool query();
template <typename T> bool get_key(T* val);
void print_rad2deg(std::vector<double> q);

//dxl util
void writeAll(std::vector<int> id_vec, std::vector<double> q, double time=0);
void writeAll(std::vector<double> q, double time=0, bool gripper=false);
void writeAll_deg(std::vector<double> q, double time=0, bool gripper=false);

//sample program util
void dxl_exit();
void rest_pos();
inline void init_pos();
inline void start_grasp();
inline void stop_grasp();
void get_fk();
void try_ik();
void teach_pos();
void set_vel();
void set_angle(int id);
void increase(int id);
void decrease(int id);


////
// Functions
////

void clear() {
  std::string command = "clear";
  system(command.c_str());
}

int getch()        // Get input key
{
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  if (ch==10) ch = getch();
  return ch;
}

void wait_key() {
  std::cout << std::endl << "(Press any key to finish)" << std::endl;
  getch();
}

bool query() {
  int tgg=0;
  do {
    std::cout << "<y/n>: ";
    int c = getch();
    if (c==ESC) exit(EXIT_SUCCESS);
    if (c==YES  || c==YES_C) tgg=1;
    if (c==NO || c==NO_C) tgg=2;
  } while(!tgg);

  std::cout << std::endl;
  if(tgg==1) return true;
  else return false;
}

template <typename T>
bool get_key(T* val) {
  std::string input;
  std::cin >> input;
  int c = (int)input[0];

  if (c==ESC) exit(EXIT_SUCCESS);
  if (((48 <= c) && (c <= 57)) || (c==45)) /* Is number of minus sign */  {    
    if(std::is_floating_point<T>::value) *val = std::stof(input);
    else *val = std::stoi(input);
    return true;
  }
  else return false;
}

void print_rad2deg(std::vector<double> q) {
  std::cout << std::fixed << std::setprecision(2);
  for (int i=0; i<q.size(); i++) 
    std::cout << std::setw(8) << std::setfill(' ') << rad2deg(q[i]) << " ";
  std::cout << std::endl << std::defaultfloat;
}


/* --- */

void writeAll(std::vector<int> id_vec, std::vector<double> q, double time) {
  std::vector<int> present_vel = getVelAll(id_vec);
  std::vector<int> overflow;
  std::vector<int> goal_vel = onTime(id_vec, q, time, &overflow);
  clock_t elapsed_time;
  int warning_threshold = 50;
  bool halt = false, warning = false;

  for(int i=0; i<overflow.size(); i++)
    std::cout << "ID " << overflow[i] << " exceeds max velocity" << std::endl;
  for(int i=0; i<goal_vel.size(); i++) 
    if(goal_vel[i] > warning_threshold) warning=true;
  if(warning) {
      std::cout << "Some actuator's velocity exceeds " << warning_threshold << std::endl;
      std::cout << "Proceed? ";
      if (!query()) halt=true;
  }

  if (!halt) {
    setVelAll(id_vec, goal_vel);
    if (time) elapsed_time = clock();
    dxl_writeAll(id_vec, rad2dxl(q), ADDR_X_GOAL_POSITION, sizeof(int32_t));
    while(isMoving()) {}

    if (time) {
      elapsed_time = clock() - elapsed_time;
      std::cout << std::endl << std::fixed;
      std::cout << std::setw(18) << "Goal Time [s]: " << std::setprecision(6) << time << std::endl;
      std::cout << std::setw(18) << "Elapsed Time [s]: " << ((float)elapsed_time)/CLOCKS_PER_SEC << std::endl;
      std::cout << std::defaultfloat;
      wait_key();
    }

    // Reset velocity to original value
    setVelAll(id_vec, present_vel);
  }
}

void writeAll(std::vector<double> q, double time, bool gripper) {
  std::vector<int> id_vec;
  for(int id=1; id<=LINK_NUM+gripper; id++)
    id_vec.push_back(id);

  writeAll(id_vec, q, time);
}

void writeAll_deg(std::vector<double> q, double time, bool gripper) {
  std::vector<int> id_vec;
  for(int id=1; id<=LINK_NUM+gripper; id++)
    id_vec.push_back(id);

  writeAll(id_vec, deg2rad(q), time);
}


/* --- */

void dxl_exit() {
  clear();
  std::cout << "Return to Rest Position? ";
  if(query()) rest_pos();
  else {
    std::cout << "Disable all? ";
    if(query()) disableAll();
  }
  exit(EXIT_SUCCESS);
}

void rest_pos() {
  writeAll_deg({0.0, -104.3, 80.8, -99.0});
  clear();
  std::cout << "Disable all? ";
  if(query()) disableAll();
}

inline void init_pos() {
  writeAll({0,0,0,0});
}
inline void start_grasp() {
  write(GRIPPER_ID, 0);
}
inline void stop_grasp() {
  write_deg(GRIPPER_ID, 35);
}

void get_fk() {
  std::vector<double> q = readAll();    // Joint State
  solveFK(q);
  std::cout << "Joint state [deg]: ";
  print_rad2deg(q);
  std::cout << "End Effector Position [mm]:" << std::endl;
  std::cout << chain[LINK_NUM].getPos() << std::endl << std::endl;
}

void try_ik() {
  std::vector<double> cur_q = readAll();     // Present Joint State
  std::vector<double> q;                     // Goal Joint State
  Vector3d goal;                             // Goal End Effector Position
  double time;                               // Goal Time

  get_fk();
  q.resize(LINK_NUM);

  for(int i=0; i<LINK_NUM; i++)
    chain[i].setAngle(cur_q[i]);

  // Get goal position and time
  do {
    std::cout << "Please input goal coordinates ( x y z ) [mm]: ";
  }while(!get_key(&goal[0]) || !get_key(&goal[1]) || !get_key(&goal[2]));

  do {
    std::cout << "Please input desired time [s] (0 for default speed): ";
  }while(!get_key(&time));

  // Solve Inverse Kinematics
  if (solveIK(goal)) {
    for(int i=0; i<LINK_NUM; i++) q[i]=chain[i].getAngle();
    std::cout << std::setw(28) << "Current joint angle [deg]: ";
    print_rad2deg(cur_q);
    std::cout << std::setw(28) << "Target joint state [deg]: ";
    print_rad2deg(q);

    std::cout << std::endl << "Proceed? ";
    if (query()) writeAll(q, time);
  }
  else {
    std::cout << "IK Fail." << std::endl;
    wait_key();
  }
}

void teach_pos() {
  bool tgg=true;
  
  while (tgg) {
    teaching_data.clearAll();
    read_file(FILENAME, [] (std::string name, std::vector<int> id_vec, std::vector<double> q)
	      { teaching_data.addNode(name, id_vec, q); } );
    clear();
    int input;
    int list_size = teaching_data.getSize();
    std::vector<int> id_test = {1,2,3,4};
    std::cout << "*** Teaching Sample Menu ***" << std::endl;
    std::cout << std::endl << " 0. Add position" << std::endl;
    teaching_data.print();
    std::cout << std::endl << "99. Exit" << std::endl << std::endl;
    do {
      std::cout << std::endl << "Please choose option number: ";
    } while(!get_key(&input) || (input>list_size && input!=99));
    clear();

    if (input == 99) tgg=false;    // Exit
    
    if (input && tgg)  /* Registered pose */  {
      node* Pose = teaching_data.getNode(input);
      std::cout << input << ". " << Pose->name << std::endl;
      int option;
      std::vector<int> id_vec = Pose->id_vec;
      std::vector<double> pos_q = Pose->q;
      std::vector<double> cur_q = readAll();
      if (id_vec == id_test) {
	std::cout << std::setw(30) << "Current joint angle [deg]: ";
	print_rad2deg(cur_q);
	std::cout << std::setw(30) << "Position joint angle [deg]: ";
	print_rad2deg(pos_q);
      }
      std::cout << std::endl;
      std::cout << "1. Go to position" << std::endl;
      std::cout << "2. Move in time" << std::endl;
      std::cout << "3. Delete position" << std::endl;
      std::cout << "4. Exit" << std::endl;
      do {
	std::cout << std::endl << "Please choose option number: ";
      } while(!get_key(&option));
      
      switch (option) {
      case 1:
	writeAll(id_vec, pos_q);
	break;
      case 2:
	double time;
	do {
	  std::cout << "Please input desired time [s]: ";
	}while(!get_key(&time));
	writeAll(id_vec, pos_q, time);
	break;
      case 3:
	std::cout << "Proceed? ";
	if (query())
	  if(!teaching_data.deleteNode(input)) {
	    std::cout << "Failed deleting." << std::endl;
	    wait_key();
	  }
	  else write_file(FILENAME, teaching_data);
	break;
      case 4:
	tgg=false;
	break;
      default:
	std::cout << "Invalid input." << std::endl;
	wait_key();
	break;
      }
    }

    if(!input)  /* Add position */  {
      if (list_size >= 98) {
	std::cout << "Too many data." << std::endl;
	wait_key();
      }
      else {
	char c;
	bool tgg=false;
	std::cout << "Disable all motors? ";
	if(query()) {
	  disableAll();
	  tgg=true;
	}
	std::cout << "Register present position? ";
	if(query()) {
	  if(tgg)enableAll();
	  std::vector<double> q = readAll();
	  std::string name;

	  std::cout << "Please input pose name: ";
	  std::cin >> name;
	  teaching_data.addNode(name, id_test, q);
	  write_file(FILENAME, teaching_data);
	  std::cout << "Registered." << std::endl;
	  wait_key();
	}
      }
    }
  }
}


void set_angle(int id) {

  int val, val_deg;
  int dxl_present_position;
  double min_angle, max_angle;
    
  getRange(id, &min_angle, &max_angle);

  //Get current position
  dxl_present_position = dxl_read(id, ADDR_X_PRESENT_POSITION, sizeof(int32_t));

  std::cout << "Current angle is " << dxl2deg(dxl_present_position) << " [deg]" << std::endl;

  //Get and write goal
  do {
    std::cout << "Please input desired angle value [deg] ";
    std::cout << std::fixed << std::setprecision(2);
    std::cout <<  "( " << min_angle << " ~ " << max_angle << " ): ";
    std::cout << std::defaultfloat;
  } while(!get_key(&val_deg) || !((val_deg >= min_angle) && (val_deg <= max_angle)) );

  write_deg(id,val_deg);
}

void set_vel() {
  int input;
  int id;
  int max_vel = 0;
  double val;
  
  std::cout << "0. Set for all" << std::endl;
  std::cout << "1. Set for ID" << std::endl << std::endl;
  do {
    std::cout << std::endl << "Please choose option number: ";
  } while(!get_key(&input) || (input!=0 && input!=1));

  if (input)  /* Set for ID */  {
    do { std::cout << "Please input motor ID: ";
    } while(!get_key(&id) || id>LINK_NUM_GRIPPER);
    max_vel = dxl_read(id, ADDR_X_VELOCITY_LIM, sizeof(int32_t));
  }

  if (!input) /* Set for all */ {
    for (id=1; id<=LINK_NUM_GRIPPER; ++id) {
      int id_vel = dxl_read(id, ADDR_X_VELOCITY_LIM, sizeof(int32_t));
      if (id_vel > max_vel) max_vel = id_vel;
    }
  }
  
  std::cout << "Max vel: " << max_vel << std::endl << std::endl;
  do {
    std::cout << "Please input desired velocity: ";
  } while(!get_key(&val));

  if (input)  dxl_write(id, val, ADDR_X_PROFILE_VEL, sizeof(int32_t));
  else setVelAll(val);
}

void increase(int id) {
  int dxl_present_position = dxl_read(id, ADDR_X_PRESENT_POSITION, sizeof(int32_t));
  double min_val, max_val, val_deg;

  getRange(id, &min_val, &max_val);

  val_deg = dxl2deg(dxl_present_position) + 10;
  if (val_deg > max_val) val_deg = max_val;
  write_deg(id,val_deg);
}
  
void decrease(int id) {
  int dxl_present_position = dxl_read(id, ADDR_X_PRESENT_POSITION, sizeof(int32_t));
  double min_val, max_val, val_deg;

  getRange(id, &min_val, &max_val);

  val_deg = dxl2deg(dxl_present_position) - 10;
  if (val_deg < min_val) val_deg = min_val;
  write_deg(id,val_deg);
}  

#endif
