#include "planner_core.h"
#include "algorithm.h"
#include "node2d.h"
namespace hybrid_astar_planner
{
    // 解决函数进入、退出的问题  解决
    // 像A*算法一样，进行路径规划 解决
    // 标注信息
    // 与move_base适配
    // 将模拟仿真的部分继承为一个launch文件
    // costmap部分仍然存在着问题
    bool HybridAStarPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                      const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        std::cout << "the start pose of planner x:" << start.pose.position.x << " y:" << start.pose.position.y << std::endl;
        std::cout << "the goal pose of planner x:" << goal.pose.position.x << " y:" << goal.pose.position.y << std::endl;
        ROS_INFO("the resolution of costmap is %lf",resolution);
        geometry_msgs::PoseStamped p;
        //检查设定的目标点参数是否合规
        if(!(checkStartPose(start) && checkgoalPose(goal))) {
            ROS_WARN("Failed to create a global plan!");
            return false;
        }
        std::cout << int(costmap->getCost(goal.pose.position.x/resolution + 20, goal.pose.position.y/resolution + 20)) << std::endl;
        if(costmap->getCost(goal.pose.position.x/resolution + 20, goal.pose.position.y/resolution + 20) > 1) {
            ROS_WARN("The goal is occupalide,please reset the goal");
            return false;
        }
        plan.clear();
        //正式将参数传入规划器中
        calculatePath(start, goal , costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), plan);
        
        
        //参数后期处理，发布到RViz上进行可视化
        // for (int i = 1; i < 10; i++) {
        //     p.pose.orientation = goal.pose.orientation;
        //     p.pose.position.x = i;
        //     p.pose.position.y = i;
        //     plan.push_back(p);
        // }
        // plan.push_back(p);
        clearPathNodes();
        publishPlan(plan);//path只能发布2D的节点
        // publishPathNodes(plan);
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
            return true;
        }
        ROS_WARN("The Goal pose is out of the map!");
        return false;
    }//end of checkgoalPose

    bool HybridAStarPlanner::calculatePath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan) {
        const unsigned char* charMap = costmap->getCharMap(); 
        int counter = 0;
        priorityQueue openSet;
        Node2D* pathNode2D = new Node2D[cells_x * cells_y]();
        Node2D *startPose = &pathNode2D[int(start.pose.position.x/resolution) *cells_x + int(start.pose.position.y/resolution)];
            //    *goalPose = &pathNode2D[int(goal.pose.position.x) *cells_x + int(goal.pose.position.y)];
        Node2D goalPose(goal.pose.position.x/resolution, goal.pose.position.y/resolution);
        startPose->setX(start.pose.position.x/resolution);
        startPose->setY(start.pose.position.y/resolution);
        startPose->setG(0);
        // goalPose->setX(goal.pose.position.x);
        // goalPose->setY(goal.pose.position.y);
        openSet.push( startPose );
        startPose->setOpenSet();
        Node2D* tmpStart;
        while(openSet.size()) {
            ++counter;
            tmpStart = openSet.top();
            // std::cout << tmpStart->getF() << std::endl;
            openSet.pop();
            std::vector<Node2D*> adjacentNodes;
            if(tmpStart->getX() == int(goal.pose.position.x/resolution) && tmpStart->getY() == int(goal.pose.position.y/resolution))
            {
                std::cout << "got a plan" << std::endl;
                nodeToPlan(tmpStart,plan);
                std::cout << counter << std::endl;
                delete [] pathNode2D;
                return true;
            }
            for (int x = tmpStart->getX() - 1; x <= tmpStart->getX() + 1; ++x) {
                for (int y = tmpStart->getY() - 1; y <= tmpStart->getY() + 1; ++y) {
                    if (charMap[x  + y* cells_x] <= 1) {
                        // std::cout << int(charMap[x * cells_x + y]) << std::endl;
                        // if(!pathNode2D[x * cells_x + y].isOpenSet()) {
                            pathNode2D[x * cells_x + y].setX(x);
                            pathNode2D[x * cells_x + y].setY(y);
                            adjacentNodes.push_back(&pathNode2D[x * cells_x + y]);
                        // }

                    }
                }
            }//end of for
            tmpStart->setClosedSet();
            //下面正式开始A*算法的核心搜索部分
            for (std::vector<Node2D*>::iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it) {
                Node2D* point = *it;
                float g;
                if (!point->isClosedSet()) {
                    g = point->calcG(tmpStart);
                    // std::cout << g <<std::endl;
                    if (!point->isOpenSet() || (g < point->getG())) {//
                        point->setPerd(tmpStart);
                        point->setG(g);
                        if(!point->isOpenSet()) {
                            // point->setH(calcH(point,goal));
                            point->calcH(goalPose);
                            point->setOpenSet();//!!!!!这里很关键
                            openSet.push(point);
                        }
                        else {
                            openSet.push(point);
                        }

                    }
                }
            }

        }
        
        delete [] pathNode2D;
        return false;
    }

    void HybridAStarPlanner::nodeToPlan(Node2D* node, std::vector<geometry_msgs::PoseStamped>& plan) {
        Node2D* tmpPtr = node;
        geometry_msgs::PoseStamped tmpPose;
        while(tmpPtr!=nullptr) {
            tmpPose.pose.position.x = tmpPtr->getX();
            tmpPose.pose.position.y = tmpPtr->getY();
            plan.push_back(tmpPose);
            tmpPtr = tmpPtr->getPerd();
        }
    }

}//end namespace hybrid_astar_planner