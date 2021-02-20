#include <iostream>
#include "planner_core.h"
#include <tf/transform_datatypes.h>
//先使用A*算法完成
//地图的信息从哪里得来？-地图信息通过costmap_ros传入。

//测试内容，程序编写不可能每一次都上机运行
//也不可能每一次都进行仿真，需要存入一张地图，直接启动进行模拟
//需要将map转化为costmap
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAStarPlanner, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_planner
{
    HybridAStarPlanner::HybridAStarPlanner():
            initialized_(false)  {
        std::cout << "hello world!" << std::endl;
    }
    void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
        // std::cout << "abc" + name <<std::endl;//test if i can print add string with another whith '+'
    }
    void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2D *costmap, std::string frame_id) {
        if(!initialized_) {
            ros::NodeHandle private_nh("~/" + name);
            frame_id_ = frame_id;
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            path_vehicles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("pathVehicle", 1);
        }
        initialized_ = true;
    }
    bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                      const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {

        std::cout << "the start pose of planner x:" << start.pose.position.x << " y:" << start.pose.position.y << std::endl;
        std::cout << "the goal pose of planner x:" << goal.pose.position.x << " y:" << goal.pose.position.y << std::endl;
        geometry_msgs::PoseStamped p;
        for (int i = 1; i < 10; i++) {
            p.pose.orientation = start.pose.orientation;//path只能发布2D的节点
            p.pose.position.x = i;
            p.pose.position.y = i;
            plan.push_back(p);
        }
        publishPlan(plan);
        publishPathNodes(plan);
    }



    void HybridAStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }
        //create a message for the plan
        geometry_msgs::PoseStamped transform_path;
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            transform_path.pose.position = path[i].pose.position;
            gui_path.poses[i] = transform_path;
            
        }

        plan_pub_.publish(gui_path);
        std::cout << "pub the path" << std::endl;
    }
    void HybridAStarPlanner::publishPathNodes(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }
        visualization_msgs::MarkerArray pathNodes;//节点数据结构，用于可视化
        visualization_msgs::Marker pathVehicle;
        pathVehicle.header.stamp = ros::Time(0);
        pathVehicle.color.r = 102.f / 255.f;
        pathVehicle.color.g = 217.f / 255.f;
        pathVehicle.color.b = 239.f / 255.f;
        pathVehicle.type = visualization_msgs::Marker::ARROW;
        pathVehicle.id = 1;
        pathVehicle.header.frame_id = "path";
        pathVehicle.scale.x = 1;
        pathVehicle.scale.y = 1;
        pathVehicle.scale.z = 1;
        pathVehicle.color.a = 0.1;
        int nodeSize = path.size();
        for(int i = 0; i<nodeSize; i++) {
            pathVehicle.header.stamp = ros::Time(0);
            pathVehicle.pose.position = path[i].pose.position;
            pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(1.0);
            // pathVehicle.pose.position.x = path[i].pose.position.x;
            // pathVehicle.pose.position.y = path[i].pose.position.y;
            // pathVehicle.pose.orientation.w = path[i].pose.orientation.w;
            // pathVehicle.pose.orientation.x = path[i].pose.orientation.x;
            // pathVehicle.pose.orientation.y = path[i].pose.orientation.y;
            // pathVehicle.pose.orientation.z = path[i].pose.orientation.z;
            std::cout << "hello !!" << std::endl;
            pathVehicle.id = i;
            pathNodes.markers.push_back(pathVehicle);
        }
        path_vehicles_pub_.publish(pathNodes);
    }

}//!end of name space hybrid_astar_planner

