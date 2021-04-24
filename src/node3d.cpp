/*********************************************************************
 *
 *  BSD 3-Clause License
 *
 *  Copyright (c) 2021, dengpw
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1 Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2 Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   3 Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 * 
 *  Author:  dengpw
 *********************************************************************/
#include "node3d.h"
#include <iostream>
#include <math.h>
// 设计启发函数
namespace hybrid_astar_planner {

void Node3D::setT(float _t) {
  if (_t>3.1415) {
    _t = _t - 6.283;
  }
  t = _t / Constants::deltaHeadingRad;
}


float Node3D::calcG() {
  float g;
  if(reverse) {
    // 如果进行的是倒车，对倒车、转向、改变方向进行惩罚
    if (reverse != perd->reverse) {                     // 对改变行驶方向进行惩罚
      g = Constants::dx[0] * Constants::penaltyCOD * Constants::penaltyReversing;
    }
    else {
      g = Constants::dx[0] * Constants::penaltyReversing; 
    }
  }
  else {
    // 如果此时位车辆前进情况，对其进行响应的代价计算
    if (reverse != perd->reverse) {                     //对方向改变进行的惩罚
      g = Constants::dx[0] * Constants::penaltyCOD;
    }
    else {
      if (t == perd->t) {
        g = Constants::dx[0];
      }
      else {
        g = Constants::dx[0] * Constants::penaltyTurning;
      }
    }
  }
  return g + perd->getG();
}

float Node3D::calcH(Node3D const *goal) {
  float dx, dy;
  dx = fabs(x - goal->x);
  dy = fabs(y - goal->y);
  h = dx +dy;
}



//###################################################
//                                        IS IN RANGE
//###################################################
bool Node3D::isInRange(const Node3D& goal) const {
  int random = rand() % 10 + 1;//产生位于[1, 10]的随机数
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;//距离的平方和在100以内则认为可达
}

//###################################################
//                                 3D NODE COMPARISON
//###################################################
//3d节点的比较函数：x和y同时相同、并且theta在一个阈值范围内时可以认为是同一个Node
bool Node3D::operator == (const Node3D& rhs) const {
  return (int)x == (int)rhs.x &&
          (int)y == (int)rhs.y &&
          (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}

} // namespace hybrid_astar_planner