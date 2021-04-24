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
#include "algorithm.h"
#include "dubins.h"
#include "ReedsShepp.h"
#include <iostream>
#include <tf/transform_datatypes.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <costmap_2d/costmap_2d.h>
namespace hybrid_astar_planner {
  
std::vector<Node2D*> gatAdjacentPoints(int cells_x, int cells_y, const unsigned char* charMap, Node2D* pathNode2D, Node2D *point ) {
  std::vector<Node2D*> adjacentNodes;
  for (int x = point->getX() - 1; x <= point->getX() + 1; ++x) {
    for (int y = point->getY() - 1; y <= point->getY() + 1; ++y) {
      if (charMap[x  + y* cells_x] < 253) {
        pathNode2D[x * cells_y + y].setX(x);
        pathNode2D[x * cells_y + y].setY(y);
        pathNode2D[x * cells_y + y].setCost(charMap[x  + y* cells_x]);
        adjacentNodes.push_back(&pathNode2D[x * cells_y + y]);
      }
    }
  }//end of for

  return adjacentNodes;
}
void nodeToPlan(Node2D* node, std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2D* costmap, std::string frame_id_) {
  Node2D* tmpPtr = node;
  geometry_msgs::PoseStamped tmpPose;
  tmpPose.header.stamp = ros::Time::now();   
  //参数后期处理，发布到RViz上进行可视化
  tmpPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      std::vector<geometry_msgs::PoseStamped> replan;
  while(tmpPtr!=nullptr) {
    costmap->mapToWorld(tmpPtr->getX(), tmpPtr->getY(), tmpPose.pose.position.x, tmpPose.pose.position.y);
    tmpPose.header.frame_id = frame_id_;
    replan.push_back(tmpPose);
    tmpPtr = tmpPtr->getPerd();
  }
  int size = replan.size();
  for (int i = 0;i < size; ++i) {
    plan.push_back(replan[size - i -1 ]);
  }
}

void  updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, 
    float* dubinsLookup, int width, int height, float inspireAstar) {
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;
  #ifdef use_ReedsShepp_heuristic
  // 假如车子可以后退，则可以启动Reeds-Shepp 算法
  if (Constants::reverse && !Constants::dubins) {
    //reeds_shepp算法还还存在问题，启用可能会造成搜寻路径增加等问题

    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd) * 1.1 + 
      0.04 * start.getCost();

  }
  #endif
  start.setH(std::max(reedsSheppCost, inspireAstar));//将两个代价中的最大值作为启发式值
}


Node3D* dubinsShot(Node3D& start, Node3D& goal, costmap_2d::Costmap2D* costmap) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::r, &path);

  unsigned int poseX, poseY;

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);
  // printf("the length of dubins %f",length);
  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  while (x <  length) {//这是跳出循环的条件之一：生成的路径没有达到所需要的长度
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(q[2]);

    // collision check
    //跳出循环的条件之二：生成的路径存在碰撞节点
    costmap->worldToMap(dubinsNodes[i].getX(), dubinsNodes[i].getY(), poseX, poseY);
    if (costmap->getCost(poseX, poseY) <= 1) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPerd(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPerd(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPerd()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      delete [] dubinsNodes;
      return nullptr;
    } 
  }
  //返回末节点，通过getPred()可以找到前一节点。
  return &dubinsNodes[i - 1];
}

Node3D* reedsSheppShot(Node3D& start, Node3D& goal, costmap_2d::Costmap2D* costmap) {

  ReedsSheppStateSpace rs_planner(Constants::r);
  double length = -1;
  unsigned int poseX, poseY;
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  std::vector<std::vector<double> > rs_path;
  rs_planner.sample(q0, q1, Constants::dubinsStepSize, length, rs_path);
  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];
  int i = 0;
  dubinsNodes[1].setPerd(&start);
  for (auto &point_itr : rs_path) {
    dubinsNodes[i].setX(point_itr[0]);
    dubinsNodes[i].setY(point_itr[1]);
    dubinsNodes[i].setT(point_itr[2]);

    //collision check
    costmap->worldToMap(dubinsNodes[i].getX(), dubinsNodes[i].getY(), poseX, poseY);
    if (costmap->getCost(poseX, poseY) < 100) {
      // set the predecessor to the previous step
      if (i > 1) {
        dubinsNodes[i].setPerd(&dubinsNodes[i - 1]);
      } 
      // else {
      //   dubinsNodes[i].setPerd(&start);
      // }
      if (&dubinsNodes[i] == dubinsNodes[i].getPerd()) {
        std::cout << "looping shot";
      }
      i++;
    } else {
      delete [] dubinsNodes;
      return nullptr;
    } 
  }
  //返回末节点，通过getPred()可以找到前一节点。
  return &dubinsNodes[i - 1];

}

} //namespace hybrid_astar_planner