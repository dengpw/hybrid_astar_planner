#include <iostream>
#include <tf/tf.h>
#include <pluginlib/class_loader.h>
#include <pluginlib_tutorials/polygon_base.h>
#include "test_plugins.h"

TestPlanner::TestPlanner(tf2_ros::Buffer &_tf):
tf(_tf){
    make_plane = n.subscribe("/move_base_simple/goal", 1, &TestPlanner::setgoal, this);//订阅目标主题，绑定响应函数,这里使用suscribe订阅目标点，当目标点刷新就重新进行路径规划
    global_planner = std::string("hybrid_astar_planner/HybridAStarPlanner");    // 定义类插件的名称，以便之后接入系统
    pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"); //导入插件
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    costmap = new costmap_2d::Costmap2DROS("global_costmap", tf);               //这个global_costmap,是它自己的名字，这里需要用param服务器给到costmap的参数，才会初始化
    std::cout << "creat the global costmap" << std::endl;
    robot_pose.header.frame_id = "base_link";                                   //指定costmap中的base_link为起始坐标
    transformStarPose();
    try
    {
        planner_=bgp_loader_.createInstance(global_planner);                    
        planner_->initialize(bgp_loader_.getName(global_planner),costmap);
    }
    catch (const pluginlib::PluginlibException &ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }
}
void TestPlanner::setgoal(const geometry_msgs::PoseStamped::ConstPtr& _goal) {
    std::cout << "receved the goal pose" <<std::endl;
    goal_pose.pose = _goal->pose;
    goal_pose.pose.orientation = _goal->pose.orientation;
    goal_pose.header = _goal->header;
    transformStarPose();
    std::cout << "I am seeing a new goal x:" << goal_pose.pose.position.x << " y:" << goal_pose.pose.position.y << std::endl;
    planner_->makePlan(start_pose, goal_pose, *planner_plan_);
}

bool TestPlanner::transformStarPose(void){
    try
    {
        start_transform = tf.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(3.0));
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    start_pose.pose.position.x = start_transform.transform.translation.x;
    start_pose.pose.position.y = start_transform.transform.translation.y;
    start_pose.pose.position.z = start_transform.transform.translation.z;
    start_pose.pose.orientation.w = start_transform.transform.rotation.w;
    start_pose.pose.orientation.x = start_transform.transform.rotation.x;
    start_pose.pose.orientation.y = start_transform.transform.rotation.y;
    start_pose.pose.orientation.z = start_transform.transform.rotation.z;
    return true;
}
TestPlanner::~TestPlanner() {
    planner_.reset();
}