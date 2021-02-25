#include <iostream>
#include "planner_core.h"
#include <tf/transform_datatypes.h>
#include <ros/node_handle.h>
//先使用A*算法完成    解决
//地图的信息从哪里得来？-地图信息通过costmap_ros传入。  解决

//测试内容，程序编写不可能每一次都上机运行      解决
//也不可能每一次都进行仿真，需要存入一张地图，直接启动进行模拟      解决
//需要将map转化为costmap    解决
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAStarPlanner, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_planner
{
    HybridAStarPlanner::HybridAStarPlanner():
            initialized_(false),costmap(NULL)  {
        std::cout << "creating the hybrid Astar planner" << std::endl;
    }
    void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
        
    }
    void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2D *_costmap, std::string frame_id) {
        if(!initialized_) {
            std::cout << "initializing the hybrid Astar planner" << std::endl;
            ros::NodeHandle nh("~/global_costmap");
            ros::NodeHandle private_nh("~/" + name);
            nh.param("resolution", resolution, 1.0);
            ROS_INFO("the resolution of costmap is %lf", _costmap->getResolution());
            frame_id_ = frame_id;
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            path_vehicles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("pathVehicle", 1);
            make_plan_srv_ = private_nh.advertiseService("make_plan", &HybridAStarPlanner::makePlanService, this);
            costmap = _costmap;
        }
        initialized_ = true;
    }//end of constructor function HybridAStarPlanner

    HybridAStarPlanner::~HybridAStarPlanner() {

    }//end of deconstructor function HybridAStarPlanner

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
            gui_path.poses[i] = transform_path;//
        }

        plan_pub_.publish(gui_path);
        std::cout << "Publish the path to Rviz" << std::endl;
    }//end of publishPlan
    void HybridAStarPlanner::publishPathNodes(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }
        visualization_msgs::Marker pathVehicle;
        
        pathVehicle.header.stamp = ros::Time(0);
        pathVehicle.color.r = 102.f / 255.f;
        pathVehicle.color.g = 217.f / 255.f;
        pathVehicle.color.b = 239.f / 255.f;
        pathVehicle.type = visualization_msgs::Marker::ARROW;
        pathVehicle.header.frame_id = frame_id_;
        pathVehicle.scale.x = 1;
        pathVehicle.scale.y = 1;
        pathVehicle.scale.z = 1;
        pathVehicle.color.a = 0.1;
        int nodeSize = path.size();
        for(int i = 0; i<nodeSize; i++) {
            pathVehicle.header.stamp = ros::Time(0);
            pathVehicle.pose = path[i].pose;
            pathVehicle.id = i;
            pathNodes.markers.push_back(pathVehicle);
        }
        
        path_vehicles_pub_.publish(pathNodes);
        
    }//end of publishPathNodes

    void HybridAStarPlanner::clearPathNodes() {
        visualization_msgs::Marker node;
        pathNodes.markers.clear();
        node.action = visualization_msgs::Marker::DELETEALL;
        node.header.frame_id = frame_id_;
        node.header.stamp = ros::Time(0);
        node.id = 0;
        node.action = 3;
        pathNodes.markers.push_back(node);
        path_vehicles_pub_.publish(pathNodes);
        std::cout << "clean the path nodes" <<std::endl;
    }
    

    bool HybridAStarPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
        makePlan(req.start, req.goal, resp.plan.poses);
        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = frame_id_;
        return true;
    }

}//end of name space hybrid_astar_planner

