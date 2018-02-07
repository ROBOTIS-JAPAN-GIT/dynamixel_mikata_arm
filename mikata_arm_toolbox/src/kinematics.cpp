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

#include <stdlib.h>
//#include "link.h"
//#include "parameters.h"
#include "mikata_arm_toolbox/kinematics.h"

/** Kinematic calculation for solving Forward and Inverse Kinematics **/

#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 3
#define IK_THRESHOLD 1    // [mm]
#define IK_GAIN 0.001
#define LIMIT_TIMES 30000

Link chain[LINK_NUM_GRIPPER];
const Matrix3d I = Matrix3d::Identity();

void setChain()        // Set basic parameters for DYNAMIXEL Mikata Arm 4DOF
{
  chain[0].setChildPos(0.0, 0.0, 76.1);
  chain[1].setChildPos(24.0, 0.0, 148.0);
  chain[2].setChildPos(0.0, 0.0, 150.0);
  chain[3].setChildPos(0, 0.0, 42.25);
  
  chain[0].setChildAtt(I);
  chain[1].setChildAtt(Y_AXIS, M_PI/2);
  chain[2].setChildAtt(I);
  chain[3].setChildAtt(I);

  chain[0].setAxis(Z_AXIS);
  chain[1].setAxis(Y_AXIS);
  chain[2].setAxis(Y_AXIS);
  chain[3].setAxis(Y_AXIS);
		   
  chain[0].setMaxAngle(3.1241);
  chain[1].setMaxAngle(1.7976);
  chain[2].setMaxAngle(1.7976);
  chain[3].setMaxAngle(1.8849);

  chain[0].setMinAngle(-3.1416);
  chain[1].setMinAngle(-2.0595);
  chain[2].setMinAngle(-1.3963);
  chain[3].setMinAngle(-1.7279);
  
  chain[0].setPos(0.0, 0.0, 0.0);
}

void solveFK(std::vector<double> q)        // Solve Forward Kinematics
{
  // Set Joint State
  for(int i=0; i<LINK_NUM; i++) chain[i].setAngle(q[i]);
  
  // Set 1st Link Attittude
  chain[0].setAtt(makeRot(chain[0].getAxis(),chain[0].getAngle()));

  // Calculate Position and Attitude for next Links
  for(int i=0; i<LINK_NUM; i++) {
    Matrix3d rot = makeRot(chain[i+1].getAxis(), chain[i+1].getAngle());
    Vector3d abs_Pos = chain[i].getPos() + chain[i].getAtt() * chain[i].getChildPos();
    Matrix3d abs_Att = chain[i].getAtt() * chain[i].getChildAtt() * rot;

    chain[i+1].setPos(abs_Pos);
    chain[i+1].setAtt(abs_Att);
  }
}

MatrixXd calcJacobian()        // Calculate base Jacobian
{
  MatrixXd J;

  for(int i=0; i<LINK_NUM; i++) {
    VectorXd col(6);
    Vector3d a = Vector3d::Zero();
    a(chain[i].getAxis()-1) = 1;
    col.tail(3) = a;
    col.head(3) = a.cross(chain[LINK_NUM].getPos() - chain[i].getPos());
    J.conservativeResize(6,J.cols() +1);
    J.col(J.cols()-1) = col;
  }

  return J;
}


bool solveIK(Vector3d goal_pos)        // Solve Inverse Kinematics for designed Goal Position
{
  VectorXd pos_diff(6);        // Difference of Actual Position and Goal Position
  std::vector<double> q;       // Actual Joint State
  int count=0;                 // Counter for executed times

  pos_diff << 0,0,0,0,0,0;
  for(int i=0; i<LINK_NUM; i++) q.push_back(chain[i].getAngle());
  solveFK(q);

  std::cout << "Calculating" << std::endl;
  
  while( ((chain[LINK_NUM].getPos() - goal_pos).norm() > IK_THRESHOLD) && count < LIMIT_TIMES) {
    
    pos_diff.head(3) = goal_pos - chain[LINK_NUM].getPos();

    MatrixXd J = calcJacobian();
    MatrixXd J_star = (J.transpose() * J).inverse() * J.transpose();
    Vector4d angle_diff;    // Joint State correction factor

    // Calculate by Jacobian Pseudo Inverse
    angle_diff= IK_GAIN * J_star * pos_diff;

    // Calculate by Jacobian Transpose
    //angle_diff= IK_GAIN * calcJacobian().transpose() * pos_diff;
    
    for(int i=0; i<LINK_NUM; i++) {
      q[i] += angle_diff(i);
      while (q[i] >= M_PI) q[i]-=2*M_PI;
      while (q[i] < -M_PI) q[i] += 2*M_PI;
      if (q[i] > chain[i].getMaxAngle()) q[i] = chain[i].getMaxAngle();
      if (q[i] < chain[i].getMinAngle()) q[i] = chain[i].getMinAngle();
    }

    solveFK(q);    // Set new Joint State and new End Effector Position

    if(count%1000==0) std::cout << "." << std::endl;;
    count++;
  }

  std::cout << std::endl;
  if (count == LIMIT_TIMES) return false;    // Calculation fails to converge
  else return true;
}

