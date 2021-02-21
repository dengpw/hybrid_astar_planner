#ifndef _TEST_PLUGINS
#define _TEST_PLUGINS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_core/base_global_planner.h>
#include <tf2_ros/transform_listener.h>

#include <boost/shared_ptr.hpp>
class TestPlanner{
    public:
        /**
         * @brief   Default constructor for the TestPlanner
         */
        TestPlanner(tf2_ros::Buffer &tf);
        ~TestPlanner();
        void setgoal(const geometry_msgs::PoseStamped::ConstPtr& _goal);

        std::vector<geometry_msgs::PoseStamped>* planner_plan_;
    private:
        bool makePlan(std::vector<geometry_msgs::PoseStamped> &plan);
        boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
        geometry_msgs::PoseStamped goal_pose;
        geometry_msgs::PoseStamped start_pose;
        bool valid_start;
        bool vaild_goal;
        ros::NodeHandle n;
        costmap_2d::Costmap2DROS* costmap;
        geometry_msgs::PoseStamped robot_pose;
        std::string global_planner;
        tf2_ros::Buffer& tf;
        ros::Subscriber make_plane;

};

#endif //!_TEST_PLUGINS