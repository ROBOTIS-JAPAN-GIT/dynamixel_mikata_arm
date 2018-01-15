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
    
