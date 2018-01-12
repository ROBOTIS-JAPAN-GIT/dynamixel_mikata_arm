#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "link.h"
#include "parameters.h"
#include <stdlib.h>

/** Kinematic calculation for solving Forward and Inverse Kinematics **/

/**
#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 3
#define IK_THRESHOLD 1    // [mm]
#define IK_GAIN 0.001
#define LIMIT_TIMES 30000
**/

extern Link chain[LINK_NUM_GRIPPER];
//const Matrix3d I = Matrix3d::Identity();

void setChain();        // Set basic parameters for DYNAMIXEL Mikata Arm 4DOF
void solveFK(std::vector<double> q);       // Solve Forward Kinematics
MatrixXd calcJacobian();        // Calculate base Jacobian
bool solveIK(Vector3d goal_pos);        // Solve Inverse Kinematics for designed Goal Position

#endif
