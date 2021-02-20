#include <iostream>
#include <boost/shared_ptr.hpp>
#include <pluginlib/class_loader.h>
#include <pluginlib_tutorials/polygon_base.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <ros/ros.h>
#include "test_plugins.h"
#include <tf/transform_listener.h>
int main(int argc, char** argv) {
    ros::init(argc,argv,"plugin_test_node");
    ros::NodeHandle n;
    Planner test;
    ros::Subscriber make_plane = n.subscribe("/move_base_simple/goal", 1, &Planner::setgoal, &test);
    std::string global_planner;
    global_planner = std::string("hybrid_astar_planner/HybridAStarPlanner");    // 定义类插件的名称，以便之后接入系统
    
    boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"); //导入插件
    tf2_ros::Buffer buffer(ros::Duration(10));                                  //ros::Duration(10)代表10秒的时间，在这里表示'tf2_ros::Buffer'实例化的对象保留10秒内的转换数据
    tf2_ros::TransformListener tf(buffer);                                      //监听转换
    // geometry_msgs::PoseStamped temp_goal;
    
    std::vector<geometry_msgs::PoseStamped>* planner_plan_;
    costmap_2d::Costmap2DROS* costmap(NULL);
    costmap = new costmap_2d::Costmap2DROS("global_costmap", buffer);           //这个global_costmap,是它自己的名字，这里需要用param服务器给到costmap的参数，才会初始化
    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.frame_id = "base_link";
    buffer.transform(robot_pose, test.start_pose, costmap->getGlobalFrameID());

    try
    {
        planner_=bgp_loader_.createInstance(global_planner);//navfn/NavfnROS
        planner_->initialize(bgp_loader_.getName(global_planner),costmap);
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }

    std::vector<geometry_msgs::PoseStamped> planner;
    planner_plan_ = &planner;
    planner_->makePlan(test.start_pose, test.goal_pose, planner);
    while(1){
      planner_->makePlan(test.start_pose, test.goal_pose, *planner_plan_);
      sleep(1);
    }
    return 0;
}