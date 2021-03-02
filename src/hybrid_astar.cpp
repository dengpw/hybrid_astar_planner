#include "node3d.h"
#include <math.h>
#include "constants.h"
#include "hybrid_astar.h"
#include <tf/transform_datatypes.h>
namespace hybrid_astar_planner {
    bool hybridAstar::calculatePath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan ) {
        ROS_INFO("Using hybrid_astar mode!");
        // static bool runOnce = false;
        // if (runOnce) {
        //     return false;
        // }
        // runOnce = true;
        std::cout << "the resolution of cost map " << costmap->getResolution() << std::endl;
        // 初始化优先队列，这里使用的是二项堆
        boost::heap::binomial_heap<Node3D*,boost::heap::compare<CompareNodes>> openSet;
        // 初始化并创建一些参数。
        // 创建charMap,charMap中存储的是地图的障碍物信息
        const unsigned char* charMap = costmap->getCharMap(); 
        float resolution = costmap->getResolution();
        unsigned int originalX, originalY, goalX, goalY;
        int counter = 0;
        int dir;
        int iPred, iSucc;
        float t, g;
        costmap->worldToMap(0, 0, originalX, originalY);
        float dx = originalX * resolution;
        float dy = originalY * resolution;

        t = tf::getYaw(start.pose.orientation);
        Node3D* startPose = new Node3D(start.pose.position.x + dx, start.pose.position.y + dy, t , 0, 0, false, nullptr);
        ROS_INFO("the x position of start node %f ,y %f" , startPose->getX(), startPose->getY());
        t = tf::getYaw(goal.pose.orientation);
        Node3D* goalPose = new Node3D(goal.pose.position.x + dx, goal.pose.position.y + dy, t , 999, 0, false, nullptr);

        std::cout << "hello world!!" <<std::endl;
        Node3D* pathNode3D = new Node3D[cells_x * cells_y * Constants::headings]();

        if (Constants::reverse) {
            dir = 6;
        }
        else {
            dir = 3;
        }

        openSet.push(startPose);
        pathNode3D[startPose->getindex(cells_x, Constants::headings)].setClosedSet();
        Node3D* tmpNode;
        while(openSet.size() && counter < Constants::iterations) {

            ++counter;
            // 根据混合A*算法，取堆顶的元素作为下查找节点
            tmpNode = openSet.top();
            openSet.pop();      //出栈
            if ( reachGoal(tmpNode, goalPose) ) {
                
                ROS_INFO("Got a plan,loop %d times!",counter);
                nodeToPlan(tmpNode, plan);
                
                delete [] pathNode3D;
                return true;
            }
            std::cout << "hello world!!A" <<std::endl;
            // 拓展tmpNode临时点目标周围的点，并且使用STL标准库的向量链表进行存储拓展点Node3D的指针数据
            std::vector<Node3D*> adjacentNodes = gatAdjacentPoints(dir, cells_x, cells_y, charMap, pathNode3D, tmpNode );   
            std::cout << "hello world!!B" <<std::endl;
            // 将 tmpNode点在pathNode3D中映射的点加入闭集合中
            pathNode3D[tmpNode->getindex(cells_x, Constants::headings)].setClosedSet();
            for (std::vector<Node3D*>::iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it) {
                // 使用stl标准库中的interator迭代器遍历相邻点
                Node3D* point = *it;
                
                iPred = point->getindex(cells_x, Constants::headings);

                // 在pathNode3D集合里映射这个点
                pathNode3D[iPred].setX(point->getX());
                pathNode3D[iPred].setY(point->getY());
                pathNode3D[iPred].setT(point->getT());
                
                
                if (!pathNode3D[iPred].isClosedSet()) {
                    g = point->calcG();
                    if (!pathNode3D[iPred].isOpenSet() || (g < pathNode3D[iPred].getG())) {//
                        // point->setPerd(tmpNode);
                        point->setG(g);
                        pathNode3D[iPred].setG(g);
                        if(!pathNode3D[iPred].isOpenSet()) {
                            point->calcH(goalPose);
                            point->setOpenSet();
                            
                            // 如果符合拓展点要求，则将此点加入优先队列中
                            openSet.push(point);
                        }
                        else {
                            openSet.push(point);
                        }

                    }
                }
            }
        }

        goalPose->setPerd(startPose);
        nodeToPlan(goalPose, plan);
        delete [] pathNode3D;
        return false;
    }


    std::vector<Node3D*> hybridAstar::gatAdjacentPoints(int dir, int cells_x, int cells_y, const unsigned char* charMap, Node3D* pathNode3D, Node3D *point ) {
        std::vector<Node3D*> adjacentNodes;
        Node3D* tmpPtr;
        float resolution = costmap->getResolution();
        float xSucc;
        float ySucc;
        float tSucc;
        unsigned int startX, startY;
        float t = point->getT();
        int index;
        float x = point->getX();
        float y = point->getY();
        unsigned int u32_x = int(x);
        unsigned int u32_y = int(y);
        for(int i = 0; i < dir; i++) {
            if (i < 3) {
                xSucc = x + Constants::dx[i] * cos(t) - Constants::dy[i] * sin(t);
                ySucc = y + Constants::dx[i] * sin(t) + Constants::dy[i] * cos(t);
            }
            else {
                xSucc = x - Constants::dx[i - 3] * cos(t) - Constants::dy[i - 3] * sin(t);
                ySucc = y - Constants::dx[i - 3] * sin(t) + Constants::dy[i - 3] * cos(t);
            }
            if( costmap->worldToMap(xSucc, ySucc, startX, startY)) {
                if (charMap[int(xSucc/resolution) + int(ySucc/resolution)* cells_y] <= 1) {
                    index = calcIndix(xSucc, ySucc, cells_x, t + Constants::dt[i]);
                    if (i<3) {
                        tmpPtr = new Node3D(xSucc, ySucc, t + Constants::dt[i], 999, 0, false,point);
                    }
                    else {
                        tmpPtr = new Node3D(xSucc, ySucc, t - Constants::dt[i-3], 999, 0, true,point);//point->getG()
                    }
                    adjacentNodes.push_back(tmpPtr);

                }
            }

        }

        return adjacentNodes;
    }

    bool hybridAstar::reachGoal(Node3D* node, Node3D* goalPose) {
        float nodeX = node->getX();
        float nodeY = node->getY();
        float goalX = goalPose->getX();
        float goalY = goalPose->getY();
        if ((nodeX < (goalX + point_accuracy) && nodeX > (goalX - point_accuracy)) && \
            (nodeY < (goalY + point_accuracy) && nodeY > (goalY - point_accuracy)) ) {
                if (node->getT() < goalPose->getT()+theta_accuracy && \
                    node->getT() > goalPose->getT()-theta_accuracy) {
                        return true;
                    }
            }
        return false;
    }

    int hybridAstar::calcIndix(float x, float y, int cells_x, float t) {
        return (int(x) * cells_x + int(y)) * Constants::headings + int(t / Constants::headings);
    }

    void hybridAstar::nodeToPlan(Node3D* node, std::vector<geometry_msgs::PoseStamped>& plan) {
        Node3D* tmpPtr = node;
        geometry_msgs::PoseStamped tmpPose;
        std::vector<geometry_msgs::PoseStamped> replan;
        float resolution = costmap->getResolution();
        unsigned int originalX,originalY;
        costmap->worldToMap(0, 0, originalX, originalY);//获取源点坐标与costMap中存在的偏移
        float dx = originalX * resolution;
        float dy = originalY * resolution;
        // int size = plan.size();
        // replan.resize(size);
        tmpPose.header.stamp = ros::Time::now();   
        //参数后期处理，发布到RViz上进行可视化
        // float resolution = costmap->getResolution();
        while(tmpPtr!=nullptr) {
            // #ifdef TEST
            tmpPose.pose.position.x = tmpPtr->getX() - dx;
            tmpPose.pose.position.y = tmpPtr->getY() - dy;
            // #else
            // costmap->mapToWorld(tmpPtr->getX()/resolution, tmpPtr->getY()/resolution, tmpPose.pose.position.x, tmpPose.pose.position.y);
            // #endif
            tmpPose.header.frame_id = frame_id_;
            tmpPose.pose.orientation = tf::createQuaternionMsgFromYaw(tmpPtr->getT());
            replan.push_back(tmpPose);
            tmpPtr = tmpPtr->getPerd();

        }
        int size = replan.size();
        for (int i = 0;i < size; ++i) {
            plan.push_back(replan[size - i -1 ]);
        }

    }

}//end of namespace hybrid_astar_planner
