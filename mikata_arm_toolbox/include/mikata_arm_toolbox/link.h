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

#ifndef __LINK_H
#define __LINK_H

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace Eigen;

/** Link class definition for Kinematic calculation **/

Matrix3d makeRot(int axis, double angle); // Rotation Matrix for designed angle and axis

class Link {
private:
  Vector3d abs_pos;            // Absolute Position
  Matrix3d abs_att;            // Absolute Attittude
  Vector3d child_rel_pos;      // Child link Position seen from this link when angle is zero
  Matrix3d child_rel_att;      // Child link Attittude seen from this link when angle is zero
  double angle;                // Joint angle
  double max_angle;            // Max joint angle
  double min_angle;            // Min joint angle
  int axis;                    // Joint rotation axis

public:

  //Get values
  
  Vector3d getPos() {
    return abs_pos;
  }
  Vector3d getChildPos() {
    return child_rel_pos;
  }
  Matrix3d getAtt() {
    return abs_att;
  }
  Matrix3d getChildAtt() {
    return child_rel_att;
  }
  double getAngle() {
    return angle;
  }
  double getMaxAngle() {
    return max_angle;
  }
  double getMinAngle() {
    return min_angle;
  }
  int getAxis() {
    return axis;
  }


  //Set values
  
  void setPos(double px, double py, double pz) {
    abs_pos << px, py, pz;
  }
  void setPos(const Vector3d Pos) {
    abs_pos = Pos;
  }
  void setChildPos(double px, double py, double pz) {
    child_rel_pos << px, py, pz;
  }
  void setAtt(const Matrix3d Att) {
    abs_att = Att;
  }
  void setChildAtt(const Matrix3d Att) {
    child_rel_att = Att;
  }
  void setChildAtt(int flag, double a) {
    child_rel_att = makeRot(flag, a);
  }
  void setAngle(double a) {
    angle = a;
  }
  void setMaxAngle(double a) {
    max_angle = a;
  }
  void setMinAngle(double a) {
    min_angle = a;
  }
  void setAxis(int a) {
    axis = a;
  }

};

Matrix3d makeRot(int axis, double angle) {
  Matrix3d Att;
  
  if (axis==1 /* x */)
    Att <<
      1, 0, 0,
      0, cos(angle), -sin(angle),
      0, sin(angle), cos(angle);
  
  if (axis==2 /* y */)
    Att <<
      cos(angle), 0, sin(angle),
      0, 1, 0,
      -sin(angle), 0, cos(angle);
  
  if (axis==3 /* z */) 
    Att <<
      cos(angle), -sin(angle), 0,
      sin(angle), cos(angle), 0,
      0, 0, 1;

  return Att;
}

#endif
