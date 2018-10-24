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

#include <iostream>
#include <sstream>
#include <stdexcept>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "mikata_arm_toolbox/dxl_util.h"
#include "mikata_arm_toolbox/parameters.h"

/** Basic utility using DynamixelSDK **/

////
// Definitions
////

#define PROTOCOL_VERSION            2.0
#define BAUDRATE                    1000000
#define DEVICENAME                  "/dev/ttyUSB0"

////
// Global Variables
////

// Package and Port Handler
dynamixel::PortHandler  *portHandler;
dynamixel::PacketHandler *packetHandler;


// Error Handler
int dxl_comm_result = COMM_TX_FAIL;
uint8_t dxl_error = 0;
bool dxl_sync_result = false;

////
// Functions
////

void dxl_setup()        // Basic setup
{
  // Initialize Handler instances
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler->openPort())
    std::cout << "Suceeded to open the port" << std::endl;
  else 
    throw std::runtime_error("Could not open the port. Please make sure the device is registered as /dev/ttyUSB0 and has permission to access.");

  // Change port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
    std::cout << "Succeeded to change the baudrate" << std::endl;
  else 
    throw std::runtime_error("Failed to change the baudrate");
}

void pingAll()        // Check connection with Dynamixels
{
  for (int id=1; id<=LINK_NUM_GRIPPER; id++) {
    if (packetHandler->ping(portHandler, id, &dxl_error)) {
      std::cout << "ERROR: Could not connect to ID " << id << std::endl;
      check_error(COMM_SUCCESS, dxl_error, "Could not connect to DYNAMIXEL");
    }
  }
}

void check_error(int dxl_res, uint8_t error, std::string msg)        // Handle Error Package
//default: msg=""
{
  if (dxl_res != COMM_SUCCESS) {
    printf("%s\n", packetHandler->getTxRxResult(dxl_res));
    throw std::runtime_error(msg.c_str());    
  }
  else if (error != 0) {
    printf("%s\n", packetHandler->getRxPacketError(error));
    throw std::runtime_error(msg.c_str());
  }
}

void check_error(bool res, std::string msg)        // Handle Error Package
//default: msg=""
{
  if (res != true)
    throw std::runtime_error(msg.c_str());
}

int dxl_read(int id, int addr, int size)        // Read data from address
{
  std::stringstream ss;
  ss << "Could not read from Control Table No." << addr;  // error message
  
  if(size==1) /* int8_t */ {
    int8_t val;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, (uint8_t*)&val, &dxl_error);
    check_error(dxl_comm_result, dxl_error, ss.str());
    return (int)val;
  }

  if(size==2) /* int16_t */ {
    int16_t val;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, (uint16_t*)&val, &dxl_error);
    check_error(dxl_comm_result, dxl_error, ss.str());
    return (int)val;
  }

  if(size==4) /* int32_t */ {
    int32_t val;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, addr, (uint32_t*)&val, &dxl_error);
    check_error(dxl_comm_result, dxl_error, ss.str());
    return (int)val;
  }

  else throw std::runtime_error("Cannot read from cast. Please use size of int8_t(=1), int16_t(=2) or int32_t(=4).");
}

std::vector<int> dxl_readAll(std::vector<int> id_vec, int addr, int size)        // SyncRead from designed IDs
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, addr, size);
  for(int i=0; i<id_vec.size(); i++) {
    dxl_sync_result = groupSyncRead.addParam(id_vec[i]);
    check_error(dxl_sync_result, "SyncRead addParam Fail");
  }
  
  dxl_comm_result = groupSyncRead.txRxPacket();
  check_error(dxl_comm_result, 0, "SyncRead Fail");
  
  for(int i=0; i<id_vec.size(); i++) {
    dxl_sync_result = groupSyncRead.isAvailable(id_vec[i], addr, size);
    check_error(dxl_sync_result, "ID Not Available to SyncRead");
  }

  std::vector<int> val;
  for(int i=0; i<id_vec.size(); i++) {
    val.push_back(groupSyncRead.getData(id_vec[i], addr, size));
  }
  return val;
}

std::vector<int> dxl_readAll(int addr, int size, bool gripper)        // SyncRead from all IDs
//default gripper=true
{
  std::vector<int> id_vec;
  for(int id=1; id<=LINK_NUM+gripper; id++)
    id_vec.push_back(id);

  return dxl_readAll(id_vec, addr, size);
}

void dxl_write(int id, int val, int addr, int size)        // Write data to address
{
  std::stringstream ss;
  ss << "Could not write to Control Table No." << addr; // error message
  
  if(size==1) /* int8_t */ 
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, val, &dxl_error);
  else
    if(size==2) /* int16_t */
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, val, &dxl_error);
    else
      if(size==4) /* int32_t */
	dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, val, &dxl_error);
      else throw std::runtime_error("Cannot read from cast. Please use size of int8_t(=1), int16_t(=2) or int32_t(=4).");

  check_error(dxl_comm_result, dxl_error, ss.str());
}

void dxl_writeAll(std::vector<int> id_vec, std::vector<int> val_vec, int addr, int size)        // SyncWrite designed values to designed IDs
{
  if(id_vec.size() != val_vec.size())
    throw std::runtime_error("Sizes mismatch.");
  if(size!=1 && size!=2 && size!=4)
    throw std::runtime_error("Cannot read from cast. Please use size of int8_t(=1), int16_t(=2) or int32_t(=4).");
  
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, addr, size);

  for(int i=0; i<id_vec.size(); i++) {
    dxl_sync_result = groupSyncWrite.addParam(id_vec[i], (uint8_t*)&val_vec[i]);
    check_error(dxl_sync_result, "SyncWrite addParam Fail");
  }
    
  dxl_comm_result = groupSyncWrite.txPacket();
  check_error(dxl_comm_result, 0, "SyncWrite Fail");

  groupSyncWrite.clearParam();
}

void dxl_writeAll(std::vector<int> id_vec, int val, int addr, int size)        // SyncWrite same value to designed IDs
{
  std::vector<int> val_vec;
  for(int i=0; i<id_vec.size(); i++)
    val_vec.push_back(val);

  dxl_writeAll(id_vec, val_vec, addr, size);
}
  
void dxl_writeAll(std::vector<int> val_vec, int addr, int size, bool gripper)        // SyncWrite designed values to all IDs
//default gripper=true
{
  std::vector<int> id_vec;
  for(int id=1; id<=LINK_NUM+gripper; id++)
    id_vec.push_back(id);

  dxl_writeAll(id_vec, val_vec, addr, size);
}

void dxl_writeAll(int val, int addr, int size, bool gripper)        // SyncWrite same value to all IDs
//default gripper=true
{
  std::vector<int> val_vec;
  for(int i=0; i<LINK_NUM+gripper; i++)
    val_vec.push_back(val);

  dxl_writeAll(val_vec, addr, size, gripper);
}

