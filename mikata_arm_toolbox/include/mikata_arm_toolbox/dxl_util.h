#ifndef __DXL_UTIL_H
#define __DXL_UTIL_H

#include <iostream>
#include <sstream>
#include <stdexcept>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "parameters.h"

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
extern dynamixel::PortHandler  *portHandler;
extern dynamixel::PacketHandler *packetHandler;


// Error Handler
extern int dxl_comm_result;
extern uint8_t dxl_error;
extern bool dxl_sync_result;

////
// Function List
////

void dxl_setup();
void pingAll();
void check_error(int res, uint8_t error, std::string msg="");
void check_error(bool res, std::string msg="");
int dxl_read(int id, int addr, int size);
std::vector<int> dxl_readAll(std::vector<int> id_vec, int addr, int size);
std::vector<int> dxl_readAll(int addr, int size, bool gripper=true);
void dxl_write(int id, int val, int addr, int size);
void dxl_writeAll(std::vector<int> id_vec, std::vector<int> val_vec, int addr, int size);
void dxl_writeAll(std::vector<int> id_vec, int val, int addr, int size);
void dxl_writeAll(std::vector<int> val_vec, int addr, int size, bool gripper=true);
void dxl_writeAll(int val, int addr, int size, bool gripper=true);

#endif
