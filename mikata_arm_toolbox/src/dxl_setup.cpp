#include "dxl_util.h"
#include "dxl_ctrl_table.h"

/** Write basic configuration to Mikata Arm 4DOF 's DYNAMIXELs **/

#define ADDR_X_MOVING_THRESHOLD  24

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

int main() {
  int vel_lim = VELOCITY_LIM;
  int mov_thre = MOVING_THRESHOLD;
  int rev_mode = REVERSE_MODE;  
  std::vector<int> max_pos = {ID_1_MAX_POS, ID_2_MAX_POS, ID_3_MAX_POS, ID_4_MAX_POS, ID_5_MAX_POS};
  std::vector<int> min_pos = {ID_1_MIN_POS, ID_2_MIN_POS, ID_3_MIN_POS, ID_4_MIN_POS, ID_5_MIN_POS};
  int offset = ID_5_OFFSET;
  
  dxl_setup();
  pingAll();
  
  dxl_writeAll(vel_lim, ADDR_X_VELOCITY_LIM, sizeof(int32_t));
  dxl_writeAll(mov_thre, ADDR_X_MOVING_THRESHOLD, sizeof(int32_t));
  dxl_writeAll(max_pos, ADDR_X_MAX_POSITION, sizeof(int32_t));
  dxl_writeAll(min_pos, ADDR_X_MIN_POSITION, sizeof(int32_t));
  
  dxl_write(GRIPPER_ID, rev_mode, ADDR_X_DRIVE_MODE, sizeof(int8_t));
  dxl_write(GRIPPER_ID, offset, ADDR_X_HOMING_OFFSET, sizeof(int32_t));

  std::cout << "Succedded on setting basic parameters." << std::endl;
  return 0;
}
