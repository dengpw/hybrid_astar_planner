#ifndef _TEST_PLUGINS
#define _TEST_PLUGINS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
class Planner{
    public:
        void setgoal(const geometry_msgs::PoseStamped::ConstPtr& _goal);
        geometry_msgs::PoseStamped goal_pose;
        geometry_msgs::PoseStamped start_pose;
    private:
        bool valid_start;
        bool vaild_goal;

};

#endif //!_TEST_PLUGINS