#include <iostream>
#include "planner_core.h"
#include <tf/transform_datatypes.h>
#include <ros/node_handle.h>
#include "astar.h"
#include "hybrid_astar.h"
// 尝试编写Hybrid A*算法完成路径规划 解决
// 优化Hybrid A*算法
// 注释！！！
PLUGINLIB_EXPORT_CLASS(hybrid_astar_planner::HybridAStarPlanner, nav_core::BaseGlobalPlanner)

namespace hybrid_astar_planner
{

    HybridAStarPlanner::HybridAStarPlanner():
            initialized_(false),costmap(NULL),resolution(1.0)  {
        std::cout << "creating the hybrid Astar planner" << std::endl;
    }


    void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
        
    }


    void HybridAStarPlanner::initialize(std::string name, costmap_2d::Costmap2D *_costmap, std::string frame_id) {
        if(!initialized_) {
            //
            ROS_INFO("initializing the hybrid Astar planner");
            // 订阅global_costmap的内容，以便获取参数
            ros::NodeHandle nh("~/global_costmap");
            ros::NodeHandle nh2("~/");
            ros::NodeHandle private_nh("~/" + name);

            nh.param("resolution", resolution, 1.0);
            ROS_INFO("the resolution of costmap is %lf",resolution);
            nh2.param("use_hybrid_astar", use_hybrid_astar, true);
            costmap = _costmap;
            frame_id_ = frame_id;
            //  初始化发布路径的主题
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            path_vehicles_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("pathVehicle", 1);

            make_plan_srv_ = private_nh.advertiseService("make_plan", &HybridAStarPlanner::makePlanService, this);
            
        }
        initialized_ = true;
    }//end of constructor function HybridAStarPlanner

    HybridAStarPlanner::~HybridAStarPlanner() {

    }//end of deconstructor function HybridAStarPlanner


    bool HybridAStarPlanner::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
        makePlan(req.start, req.goal, resp.plan.poses);
        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = frame_id_;
        return true;
    }


    bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                      const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped>& plan) {
        
        std::cout << "the start pose of planner x:" << start.pose.position.x << " y:" << start.pose.position.y << std::endl;
        std::cout << "the goal pose of planner x:" << goal.pose.position.x << " y:" << goal.pose.position.y << std::endl;
        Expander* _planner;
        if (use_hybrid_astar) {
            _planner = new hybridAstar(frame_id_,costmap);
        }
        else {
            _planner = new astar(frame_id_,costmap);
        }
        
        // 
        //检查设定的目标点参数是否合规
        if(!(checkStartPose(start) && checkgoalPose(goal))) {
            ROS_WARN("Failed to create a global plan!");
            return false;
        }
        plan.clear();
        //正式将参数传入规划器中
        _planner->calculatePath(start, goal , costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), plan);
        //参数后期处理，发布到RViz上进行可视化
        clearPathNodes();
        publishPlan(plan);//path只能发布2D的节点
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
            if (costmap->getCost( goalx, goaly ) > 2) {
                ROS_WARN("The Goal pose is occupied , please reset the goal!");
                return false;
            }
            return true;
        }
        ROS_WARN("The Goal pose is out of the map!");
        return false;
    }//end of checkgoalPose

}//end of name space hybrid_astar_planner

