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

#include "mikata_arm_toolbox/dxl_util.h"
#include "mikata_arm_toolbox/dxl_ctrl_table.h"

/** Write basic configuration to Mikata Arm 4DOF 's DYNAMIXELs **/

#define POSITION_CONTROL_MODE   3
#define VELOCITY_LIM       100
#define MOVING_THRESHOLD   5
#define REVERSE_MODE       1
#define ID_1_MAX_POS       4095
#define ID_1_MIN_POS       0
#define ID_2_MAX_POS       3400
#define ID_2_MIN_POS       860
#define ID_3_MAX_POS       2970
#define ID_3_MIN_POS       910
#define ID_4_MAX_POS       3280
#define ID_4_MIN_POS       920
#define ID_5_MAX_POS       3090
#define ID_5_MIN_POS       1910
#define ID_5_OFFSET        -138

void display_current_dxl_settings();

int main() {
  int vel_lim = VELOCITY_LIM;
  int mov_thre = MOVING_THRESHOLD;
  int rev_mode = REVERSE_MODE;  
  std::vector<int> max_pos = {ID_1_MAX_POS, ID_2_MAX_POS, ID_3_MAX_POS, ID_4_MAX_POS, ID_5_MAX_POS};
  std::vector<int> min_pos = {ID_1_MIN_POS, ID_2_MIN_POS, ID_3_MIN_POS, ID_4_MIN_POS, ID_5_MIN_POS};
  int offset = ID_5_OFFSET;
  
  dxl_setup();
  pingAll();
  
  std::cout << "DXL_setup_mikata_arm_position_control_mode" << std::endl;
  std::cout << "checking the DXLs settings." << std::endl;
  std::cout << "  current_DXL_settings" << std::endl;
  display_current_dxl_settings();

  dxl_writeAll(POSITION_CONTROL_MODE, ADDR_X_OPERATING_MODE, sizeof(int8_t));
  dxl_writeAll(vel_lim, ADDR_X_VELOCITY_LIM, sizeof(int32_t));
  dxl_writeAll(mov_thre, ADDR_X_MOVING_THRESHOLD, sizeof(int32_t));
  dxl_writeAll(max_pos, ADDR_X_MAX_POSITION, sizeof(int32_t));
  dxl_writeAll(min_pos, ADDR_X_MIN_POSITION, sizeof(int32_t));
  
  dxl_write(GRIPPER_ID, rev_mode, ADDR_X_DRIVE_MODE, sizeof(int8_t));
  dxl_write(GRIPPER_ID, offset, ADDR_X_HOMING_OFFSET, sizeof(int32_t));

  std::cout << "Succedded on setting basic parameters." << std::endl;
  std::cout << "  new_DXL_settings:" << std::endl;
  display_current_dxl_settings();
  return 0;
}

void display_current_dxl_settings() {
    std::vector<int> velocity_limits;
    std::vector<int> operating_modes;
    operating_modes = dxl_readAll(ADDR_X_OPERATING_MODE, sizeof(int8_t), true);
    velocity_limits = dxl_readAll(ADDR_X_VELOCITY_LIM, sizeof(int32_t), true);

    for(int i=0; i<LINK_NUM_GRIPPER; i++) {
      int id=i+1;
      std::cout << "    joint ID: " << id 
            << ", operating_mode: " << operating_modes[i]
            << ", velocity_limit: " << velocity_limits[i]
            << std::endl;
    }
}

