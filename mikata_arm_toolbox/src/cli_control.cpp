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

#include "mikata_arm_toolbox/cli_core.h"

using namespace std;

void print_table();

std::vector<double> q;
std::vector<double> link_x_pos;
std::vector<double> link_y_pos;
std::vector<double> link_z_pos;
std::vector<int> link_vel;
std::vector<int> link_acel;

int main()
{
  dxl_setup();
  pingAll();

  if (!dxl_read(GRIPPER_ID, ADDR_X_DRIVE_MODE, sizeof(int8_t)))
    throw std::runtime_error("Please run dxl_setup before execution.");

  setChain();
  enableAll();
  setVelAll(20);
  setAcelAll(2, false);
  dxl_write(GRIPPER_ID, 30, ADDR_X_PROFILE_VEL, sizeof(int32_t));
  dxl_write(GRIPPER_ID, 250, ADDR_X_P_GAIN, sizeof(int16_t));
  
  while(1) {
    
    //Clear Parameters
    clear();
    q.clear();
    link_x_pos.clear();
    link_y_pos.clear();
    link_z_pos.clear();
    link_vel.clear();
    link_acel.clear();

    //Set Parameters
    q = readAll(true);
    link_vel = getVelAll();
    link_acel = dxl_readAll(ADDR_X_PROFILE_ACEL, sizeof(int32_t));
    solveFK(q);
    for(int i=0; i<=LINK_NUM; i++) {
      int id = i+1;
      Vector3d val = chain[i].getPos();
      link_x_pos.push_back(val[0]);
      link_y_pos.push_back(val[1]);
      link_z_pos.push_back(val[2]);
    }

    //Print Command Table
    print_table();

    //Get Command
    int c = getch();

    switch(c)
      {
      case 48:       /* 0 */    // Update table
	break;
      case 49:       /* 1 */    // Set ID 1 angle
	clear();
	set_angle(1);
	break;
      case 50:       /* 2 */    // Set ID 2 angle
	clear();
	set_angle(2);
	break;
      case 51:       /* 3 */    // Set ID 3 angle
	clear();
	set_angle(3);
	break;
      case 52:       /* 4 */    // Set ID 4 angle
	clear();
	set_angle(4);
	break;
      case 53:       /* 5 */    // Set ID 5 angle
	clear();
	set_angle(5);
	break;
      case 113:       /* q */    // Increase ID 1 angle
	increase(1);
	break;
      case 119:       /* w */    // Increase ID 2 angle
	increase(2);
	break;
      case 101:       /* e */    // Increase ID 3 angle
	increase(3);
	break;
      case 114:       /* r */    // Increase ID 4 angle
	increase(4);
	break;
      case 116:       /* t */    // Increase ID 5 angle
	increase(5);
	break;
      case 121:       /* y */    // Decrease ID 1 angle
	decrease(1);
	break;
      case 117:       /* u */    // Decrease ID 2 angle
	decrease(2);
	break;
      case 105:       /* i */    // Decrease ID 3 angle
	decrease(3);
	break;
      case 111:       /* o */    // Decrease ID 4 angle
	decrease(4);
	break;
      case 112:       /* p */    // Decrease ID 5 angle
	decrease(5);
	break;
      case 97:        /* a */    // Inverse Kinematics sample
	clear();
	try_ik();
	break;
      case 115:       /* s */    // Teaching Sample
	teach_pos();
	break;
      case 100:       /* d */    // Move to Initial Position
	init_pos();
	break;
      case 102:       /* f */    // Move to Rest Position
	rest_pos();
	break;
      case 103:       /* g */    // Start grasp
	start_grasp();
	break;
      case 104:       /* h */    // Stop grasp
	stop_grasp();
	break;
      case 106:       /* j */    // Set velocity
	clear();
	set_vel();
	break;
      case 110:       /* n */    // Enable Torque for all IDs
	enableAll();
	break;
      case 109:       /* m */    // Disable Torque for all IDs
	disableAll();
	break;

      case ESC:                  // Exit program
	dxl_exit();
      }
    cout << endl;
  }
}
  
  
void print_table() {
  cout << setw(55) << ".--------------------------------------." << endl;
  cout << setw(55) << "|  DYNAMIXEL MikataArm Sample Program  |" << endl;
  cout << setw(55) << "'--------------------------------------'" << endl;
  cout << endl << endl;

  // ID
  cout << setw(13) << "ID:"
       << setw(10) << "1"
       << setw(10) << "2"
       << setw(10) << "3"
       << setw(10) << "4"
       << setw(10) << "5";
  cout << endl << endl;

  // Angle
  cout << fixed << setprecision(2);
  cout << setw(13) << "Angle [deg]:"
       << setw(12) << rad2deg(q[0])
       << setw(10) << rad2deg(q[1])
       << setw(10) << rad2deg(q[2])
       << setw(10) << rad2deg(q[3])
       << setw(10) << rad2deg(q[4]);  
  cout << endl << endl << defaultfloat;

  // Position
  cout << fixed << setprecision(3);
  cout << setw(13) << "Pos [mm]:" << " (x)"
       << defaultfloat
       << setw(6) << link_x_pos[0]
       << setw(10) << link_x_pos[1]
       << fixed
       << setw(12) << link_x_pos[2]
       << setw(10) << link_x_pos[3]
       << setw(10) << link_x_pos[4];
  cout << endl;
  cout << setw(13) << "" << " (y)"
       << defaultfloat
       << setw(6) << link_y_pos[0]
       << setw(10) << link_y_pos[1]
       << fixed
       << setw(12) << link_y_pos[2]
       << setw(10) << link_y_pos[3]
       << setw(10) << link_y_pos[4];
  cout << endl;
  cout << setw(13) << "" << " (z)"
       << defaultfloat
       << setw(6) << link_z_pos[0]
       << fixed
       << setw(12) << link_z_pos[1]
       << setw(10) << link_z_pos[2]
       << setw(10) << link_z_pos[3]
       << setw(10) << link_z_pos[4];
  cout << endl << endl << defaultfloat;

  // Velocity
  cout << setw(13) << "Vel :"
       << setw(11) << link_vel[0]
       << setw(10) << link_vel[1]
       << setw(10) << link_vel[2]
       << setw(10) << link_vel[3]
       << setw(10) << link_vel[4];
  cout << endl;

  cout << setw(13) << "Accel :"
       << setw(11) << link_acel[0]
       << setw(10) << link_acel[1]
       << setw(10) << link_acel[2]
       << setw(10) << link_acel[3]
       << setw(10) << link_acel[4];
  cout << endl << endl;

  cout << setw(53) << "------------------------------------";
  cout << endl;

  // Controls
  cout << setw(31) << "0. Update table" << setw(24) << "Esc. Exit\t" << endl << endl;
  cout << "1~5. Set angle\t \t" << "q~t. Increase angle\t" << "y~p. Decrease angle\t" << endl;
  cout << " a . IK sample\t \t" << " s . Teaching sample\t" << " d . Initial Position\t" << endl;
  cout << " f . Rest Position\t" << " g . Start grasp\t" << " h . Stop grasp\t" << endl;
  cout << " j . Set Velocity\t" << " n . Enable All\t \t" << " m . Disable All\t" << endl;

}
