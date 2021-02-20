#include <iostream>
#include "test_plugins.h"
#include <tf/tf.h>
void Planner::setgoal(const geometry_msgs::PoseStamped::ConstPtr& _goal) {
    std::cout << "receved the goal pose" <<std::endl;
    goal_pose.pose = _goal->pose;
    goal_pose.header = _goal->header;

    //float t = tf::getYaw(_goal->pose.orientation);

    std::cout << "I am seeing a new goal x:" << goal_pose.pose.position.x << " y:" << goal_pose.pose.position.y << std::endl;

//   if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
//     validGoal = true;
//     goal = *end;

//     if (Constants::manual) { plan();}

//   } else {
//     std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
//   }
}