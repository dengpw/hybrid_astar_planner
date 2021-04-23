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
#include <iostream>
#include "planner_core.h"
#include <tf/transform_datatypes.h>
#include <ros/node_handle.h>
#include "astar.h"
#include "hybrid_astar.h"

PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAStarPlanner, nav_core::BaseGlobalPlanner)//注册为类插件的声明

namespace hybrid_astar_planner {

HybridAStarPlanner::HybridAStarPlanner():
    initialized_(false),costmap(NULL),resolution(1.0) {
  std::cout << "creating the hybrid Astar planner" << std::endl;
}


void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
{
  initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}


void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2D *_costmap, std::string frame_id) {
  if(!initialized_) {
    ROS_INFO("initializing the hybrid Astar planner");
    // 订阅global_costmap的内容，以便获取参数
    ros::NodeHandle nh("~/global_costmap");
    ros::NodeHandle nh2("~/");
    ros::NodeHandle private_nh("~/" + name);
    nh2.param("use_hybrid_astar", use_hybrid_astar, true);
    if(use_hybrid_astar) {
      ROS_INFO("Using hybrid_astar mode!");
    } else {
      ROS_INFO("Using Astar mode!");
    }
    costmap = _costmap;
    frame_id_ = frame_id;
    std::cout << frame_id << std::endl;
    //  初始化发布路径的主题
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    path_vehicles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("pathVehicle", 1);
    make_plan_srv_ = private_nh.advertiseService("make_plan", &HybridAStarPlanner::makePlanService, this);
  }
  initialized_ = true;
}//end of constructor function HybridAStarPlanner

HybridAStarPlanner::~HybridAStarPlanner() {

}//end of deconstructor function HybridAStarPlanner


bool HybridAStarPlanner::makePlanService(nav_msgs::GetPlan::Request& req, 
    nav_msgs::GetPlan::Response& resp) {
  makePlan(req.start, req.goal, resp.plan.poses);
  resp.plan.header.stamp = ros::Time::now();
  resp.plan.header.frame_id = frame_id_;
  return true;
}


bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &goal, 
    std::vector<geometry_msgs::PoseStamped>& plan) {
  // std::cout << "the start pose of planner x:" << start.pose.position.x << " y:" << start.pose.position.y << std::endl;
  // std::cout << "the goal pose of planner x:" << goal.pose.position.x << " y:" << goal.pose.position.y << std::endl;
  Expander* _planner;

  // ROS_INFO("the resolution of cost map: %f ",costmap->getResolution());
  if (use_hybrid_astar) {  
    _planner = new hybridAstar(frame_id_,costmap);
  }
  else {
    _planner = new astar(frame_id_,costmap);
  }

  //检查设定的目标点参数是否合规
  if(!(checkStartPose(start) && checkgoalPose(goal))) {
    ROS_WARN("Failed to create a global plan!");
    return false;
  }
  plan.clear();
  //正式将参数传入规划器中
  if(!_planner->calculatePath(start, goal , costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), plan, path_vehicles_pub_, pathNodes)) {
    return false;
  }
  
  //参数后期处理，发布到RViz上进行可视化
  clearPathNodes();
  //path只能发布2D的节点
  publishPlan(plan);
  publishPathNodes(plan);
  return true;
}//end of makeplan

bool HybridAStarPlanner::checkStartPose(const geometry_msgs::PoseStamped &start) {
    unsigned int startx,starty;
  if (costmap->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty)) {
    return true;
  }
  ROS_WARN("The Start pose is out of the map!");
  return false;
}//end of checkStartPose

bool HybridAStarPlanner::checkgoalPose(const geometry_msgs::PoseStamped &goal) {
  unsigned int goalx,goaly;
  if (costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly)) {
    if (costmap->getCost( goalx, goaly ) > 252) {
      // std::cout << costmap->getCost(goalx, goaly) << std::endl;
      ROS_WARN("The Goal pose is out of the map! %d",costmap->getCost(goalx, goaly));
      ROS_WARN("The Goal pose is occupied , please reset the goal!");
      return false;
    }
    return true;
  }
  return false;
}//end of checkgoalPose

}//namespace hybrid_astar_planner

