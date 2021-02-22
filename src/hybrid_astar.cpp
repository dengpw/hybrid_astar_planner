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
        if(!(checkStartPose(start) && checkgoalPose(goal))) {
            ROS_WARN("Failed to create a global plan!");
            return false;
        }
        // costmap->setCost(1,1,100);
        plan.clear();
        std::cout << "get the cost of start pose" << costmap->getCost(1,1) << std::endl;

        // costmap->setCost(0,1,200);
        //costmap->resizeMap(80,80,1,0,0);
        // hybridAstarPlanner(start,start,costmap->getCharMap());






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
        return true;
    }//end of makeplan

    bool HybridAStarPlanner::checkStartPose(const geometry_msgs::PoseStamped &start) {
        unsigned int startx,starty;
        if (costmap->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty)) {
            // std::cout << "hahhahahhah" << costmap->getSizeInMetersX() << std::endl;
            return true;
        }
        
        ROS_WARN("The Start pose is out of the map!");
        return false;
    }//end of checkStartPose

    bool HybridAStarPlanner::checkgoalPose(const geometry_msgs::PoseStamped &goal) {
        unsigned int goalx,goaly;
        if (costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly)) {
            std::cout << "hahhahahhah" << costmap->getSizeInCellsX() << ", y:" << costmap->getSizeInCellsY() << std::endl;
            return true;
        }
        ROS_WARN("The Goal pose is out of the map!");
        return false;
    }//end of checkgoalPose

    bool HybridAStarPlanner::hybridAstarPlanner(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      unsigned char* path) {
        
        // for(int i = 0; i < 200; ++i) {
        //     printf("%d",path[i]);
        // }
    }

}//end namespace hybrid_astar_planner