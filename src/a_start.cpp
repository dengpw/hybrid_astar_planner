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
#include "node2d.h"
#include "astar.h"
#include <tf/transform_datatypes.h>
namespace hybrid_astar_planner {

bool astar::calculatePath(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal,
    int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan,
    ros::Publisher& pub, visualization_msgs::MarkerArray& pathNodes ) {
    
  const unsigned char* charMap = costmap->getCharMap();

  int counter = 0;                            // 记录程序搜寻节点次数
  float resolution = costmap->getResolution();// 获取代价图的分辨率

  // 使用boost库中的二项堆，优化优先队列的性能
  boost::heap::binomial_heap<Node2D*,boost::heap::compare<CompareNodes>> openSet;
  unsigned int startx, starty, goalx, goaly;
  // 坐标转换，将世界坐标转换为costmap使用的绝对坐标
  costmap->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty);
  costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly);
  Node2D* pathNode2D = new Node2D[cells_x * cells_y]();  // 与代价图等大的2Dnode栅格节点

  // 设置开始节点与结束节点指针，指针指向其对应2D节点图中的点位
  Node2D* startPose = &pathNode2D[ startx * cells_y + starty ];
  Node2D* goalPose = &pathNode2D[ goalx * cells_y + goaly ];
  
  // 设置起始节点和结束节点的坐标，以便后续进行比较计算
  goalPose->setX( goalx );
  goalPose->setY( goaly );
  startPose->setX( startx );
  startPose->setY( starty );
  startPose->setG(0);
  openSet.push( startPose );
  startPose->setOpenSet();

  Node2D* tmpStart;
  while(openSet.size()) {
    ++counter;
    tmpStart = openSet.top();
    openSet.pop();
    //如果找到目标点则返回   
    if(tmpStart->getX() == goalPose->getX() && tmpStart->getY() == goalPose->getY() ) {
      std::cout << "got a plan" << std::endl;
      nodeToPlan(tmpStart, plan);
      std::cout << counter << std::endl;
      delete [] pathNode2D;
      return true;
    }
    std::vector<Node2D*> adjacentNodes = gatAdjacentPoints(cells_x, cells_y, charMap, pathNode2D, tmpStart );    
    tmpStart->setClosedSet();

    //下面正式开始A*算法的核心搜索部分
    for (std::vector<Node2D*>::iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it) { 
      Node2D* point = *it;
      float g;
      if (!point->isClosedSet()) {
        g = point->calcG(tmpStart);
        // 在此拓展点为初次被探索时，设置此点的G值，设置其父节点。或是以其他路径到达此点的G值更小时，重新设置此点的父节点
        if (!point->isOpenSet() || (g < point->getG())) {
          point->setPerd(tmpStart);
          point->setG(g);
          if(!point->isOpenSet()) {
            point->calcH(goalPose);                 // 计算此点距离目标节点距离（作为启发值）
            point->setOpenSet();                    // 将此拓展点加入开集合中
          }
          openSet.push(point);
        }
      }
    }
  }

  delete [] pathNode2D;       //删除产生的Node2D节点
  return false;               //搜索失败
}

std::vector<Node2D*> astar::gatAdjacentPoints(int cells_x, int cells_y, const unsigned char* charMap, Node2D* pathNode2D, Node2D *point ) {
    std::vector<Node2D*> adjacentNodes;
  for (int x = point->getX() - 1; x <= point->getX() + 1; ++x) {
    for (int y = point->getY() - 1; y <= point->getY() + 1; ++y) {
      if (charMap[x  + y* cells_x] <= 1) {
        pathNode2D[x * cells_y + y].setX(x);
        pathNode2D[x * cells_y + y].setY(y);
        adjacentNodes.push_back(&pathNode2D[x * cells_y + y]);
      }
    }
  }//end of for

  return adjacentNodes;
}

void astar::nodeToPlan(Node2D* node, std::vector<geometry_msgs::PoseStamped>& plan) {
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

}// namespace hybrid_astar_planner
