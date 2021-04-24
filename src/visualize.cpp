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
#include "planner_core.h"
#include "node3d.h"
#include <tf/transform_datatypes.h>
namespace hybrid_astar_planner {
  
void HybridAStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
  if (!initialized_) {
    ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }
  //create a message for the plan
  geometry_msgs::PoseStamped transform_path;
  nav_msgs::Path gui_path;
  int size = path.size();
  gui_path.poses.resize(size);

  gui_path.header.frame_id = frame_id_;
  gui_path.header.stamp = ros::Time::now();

  // Extract the plan in world co-ordinates, we assume the path is all in the same frame
  for (unsigned int i = 0; i < size; i++) {
    transform_path.pose.position = path[i].pose.position;
    gui_path.poses[i] = transform_path;//
  }

  plan_pub_.publish(gui_path);
  // ROS_INFO("Publish the path to Rviz");
    
}//end of publishPlan


void HybridAStarPlanner::publishPathNodes(const std::vector<geometry_msgs::PoseStamped>& path) {
  if (!initialized_) {
    ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return;
  }
  visualization_msgs::Marker pathVehicle;
  int nodeSize = path.size();
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.color.r = 52.f / 255.f;
  pathVehicle.color.g = 250.f / 255.f;
  pathVehicle.color.b = 52.f / 255.f;
  pathVehicle.type = visualization_msgs::Marker::ARROW;
  pathVehicle.header.frame_id = frame_id_;
  pathVehicle.scale.x = 0.22;
  pathVehicle.scale.y = 0.18;
  pathVehicle.scale.z = 0.12;
  pathVehicle.color.a = 0.1;
  // 转化节点，并同时加上时间戳等信息
  for(int i = 0; i<nodeSize; i++) {
    pathVehicle.header.stamp = ros::Time(0);
    pathVehicle.pose = path[i].pose;
    pathVehicle.id = i;
    pathNodes.markers.push_back(pathVehicle);
  }
  // 发布这些车辆位置标记点
  path_vehicles_pub_.publish(pathNodes);
    
}//end of publishPathNodes

void HybridAStarPlanner::clearPathNodes() {
  // 初始化并配置节点为全清空模式
  visualization_msgs::Marker node;
  pathNodes.markers.clear();
  node.action = visualization_msgs::Marker::DELETEALL;
  node.header.frame_id = frame_id_;
  node.header.stamp = ros::Time(0);
  node.id = 0;
  node.action = 3;
  pathNodes.markers.push_back(node);
  path_vehicles_pub_.publish(pathNodes);
  // ROS_INFO("Clean the path nodes");
}

 void publishSearchNodes(Node3D node,ros::Publisher& pub, 
    visualization_msgs::MarkerArray& pathNodes, int i) {
  visualization_msgs::Marker pathVehicle;
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.color.r = 250.f / 255.f;
  pathVehicle.color.g = 250.f / 255.f;
  pathVehicle.color.b = 52.f / 255.f;
  pathVehicle.type = visualization_msgs::Marker::ARROW;
  pathVehicle.header.frame_id = "map";
  pathVehicle.scale.x = 0.22;
  pathVehicle.scale.y = 0.18;
  pathVehicle.scale.z = 0.12;
  pathVehicle.color.a = 0.1;
  // 转化节点，并同时加上时间戳等信息
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.pose.position.x = node.getX();
  pathVehicle.pose.position.y = node.getY();
  pathVehicle.pose.position.z = 0;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicle.id = i;
  pathNodes.markers.push_back(pathVehicle);
  
  // 发布这些车辆位置标记点
  pub.publish(pathNodes);
    
}//end of publishPathNodes   
}
