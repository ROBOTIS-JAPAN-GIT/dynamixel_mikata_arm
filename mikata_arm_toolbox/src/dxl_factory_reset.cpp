#include "dxl_util.h"

/** Factory Reset all DYNAMIXELs **/

#define OPERATION_MODE 0x02 //reset all values except ID and baudrate

int main()
{
  dxl_setup();
  pingAll();
  
  for (int id=1; id<=LINK_NUM_GRIPPER; id++) {
    std::cout << "Resetting ID " << id << std::endl;
    dxl_comm_result = packetHandler->factoryReset(portHandler, id, OPERATION_MODE, &dxl_error);
    check_error(dxl_comm_result, dxl_error);
  }

  std::cout << "Succeeded on resetting all." << std::endl;

  return 0;
}
    
