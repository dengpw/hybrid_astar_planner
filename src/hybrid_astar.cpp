#include "planner_core.h"
namespace hybrid_astar_planner
{
    // 解决函数进入、退出的问题  解决
    // 像A*算法一样，进行路径规划
    bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                      const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        std::cout << "the start pose of planner x:" << start.pose.position.x << " y:" << start.pose.position.y << std::endl;
        std::cout << "the goal pose of planner x:" << goal.pose.position.x << " y:" << goal.pose.position.y << std::endl;
        geometry_msgs::PoseStamped p;
        plan.clear();
        // publishPathNodes(plan,1);
        for (int i = 1; i < 10; i++) {
            p.pose.orientation = goal.pose.orientation;
            p.pose.position.x = i;
            p.pose.position.y = i;
            plan.push_back(p);
        }
        // path_vehicles_pub_.shutdown
        plan.push_back(p);
        clearPathNodes();
        publishPlan(plan);//path只能发布2D的节点
        publishPathNodes(plan);
    }
}