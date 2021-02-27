#include "node3d.h"
#include "constants.h"
#include "hybrid_astar.h"
#include <tf/transform_datatypes.h>
namespace hybrid_astar_planner {
    bool hybridAstar::calculatePath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan ) {
        std::cout << "Using hybrid_astar mode!" << std::endl;
        boost::heap::binomial_heap<Node3D*,boost::heap::compare<CompareNodes>> openSet;

        const unsigned char* charMap = costmap->getCharMap(); 
        unsigned int startX, startY, goalX, goalY;
        int counter = 0;
        int dir;
        int iPred, iSucc;
        float t, g;
        t = tf::getYaw(start.pose.orientation);
        Node3D* startPose = new Node3D(start.pose.position.x, start.pose.position.y, t , 0, 0, false, nullptr);
        t = tf::getYaw(goal.pose.orientation);
        Node3D* goalPose = new Node3D(goal.pose.position.x, goal.pose.position.y, t , 999, 0, false, nullptr);


        costmap->worldToMap(start.pose.position.x, start.pose.position.y, startX, startY);
        costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalX, goalY);
        Node3D* pathNode3D = new Node3D[cells_x * cells_y * Constants::headings]();
        // Node3D* startPose = &pathNode3D[ (startX * cells_x + startY) * Constants::headings ];
        // Node3D* goalPose = &pathNode3D[ (goalY * cells_x + goalY) * Constants::headings ];

        // startPose->setX(start.pose.position.x);
        // startPose->setY(start.pose.position.y);
        // t = tf::getYaw(start.pose.orientation);
        // startPose->setT(t);


        // goalPose->setX(goal.pose.position.x);
        // goalPose->setY(goal.pose.position.y);
        // t = tf::getYaw(goal.pose.orientation);
        // // std::cout << "the t of goal pose" << t << std::endl;
        // goalPose->setT(t);
        std::cout << goalPose->getT() << t << std::endl;
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
            tmpNode = openSet.top();
            openSet.pop();
            if ( int(tmpNode->getX()) == int(goalPose->getX()) && int(tmpNode->getY()) == int(goalPose->getY()) ) {
                ROS_INFO("got a plan");
                nodeToPlan(tmpNode, plan);
                std::cout << counter << std::endl;
                delete [] pathNode3D;
                return true;
            }
            std::vector<Node3D*> adjacentNodes = gatAdjacentPoints(dir, cells_x, cells_y, charMap, pathNode3D, tmpNode );    
            // tmpNode->setClosedSet();
            pathNode3D[tmpNode->getindex(cells_x, Constants::headings)].setClosedSet();
            for (std::vector<Node3D*>::iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it) {
                Node3D* point = *it;
                // ROS_INFO("hello1");
                iPred = point->getindex(cells_x, Constants::headings);
                if (!pathNode3D[iPred].isClosedSet()) {
                    g = pathNode3D[iPred].calcG(tmpNode);
                    // g = point->calcG(tmpStart);
                    // ROS_INFO("hello2");
                    if (!pathNode3D[iPred].isOpenSet() || (g < pathNode3D[iPred].getG())) {//
                        point->setPerd(tmpNode);
                        point->setG(g);
                        if(!point->isOpenSet()) {
                            point->calcH(goalPose);
                            point->setOpenSet();//
                            openSet.push(point);
                        }
                        else {
                            openSet.push(point);
                        }

                    }
                }
            }
            
            // std::cout << "the index of node : " << iPred << std::endl; 

        }

        goalPose->setPerd(startPose);
        nodeToPlan(goalPose, plan);
        delete [] pathNode3D;
        return false;
    }

    std::vector<Node3D*> hybridAstar::gatAdjacentPoints(int dir, int cells_x, int cells_y, const unsigned char* charMap, Node3D* pathNode3D, Node3D *point ) {
        std::vector<Node3D*> adjacentNodes;
        Node3D* tmpPtr;
        float xSucc;
        float ySucc;
        float tSucc;
        int t = point->getT();
        int index;
        float x = point->getX();
        float y = point->getY();
        // std::cout << "the expention of node x : " << x << " Y : " << y << std::endl;
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

            if (charMap[int(xSucc) + int(ySucc)* cells_y] <= 1) {
                index = calcIndix(xSucc, ySucc, cells_x, t + Constants::dt[i]);
                // pathNode3D[index].setX(xSucc);
                // pathNode3D[index].setY(ySucc);
                // pathNode3D[index].setT(t + Constants::dt[i]);
                if (i<3) {
                    tmpPtr = new Node3D(xSucc, ySucc, t + Constants::dt[i], point->getG(), 0, false,point);
                }
                else {
                    tmpPtr = new Node3D(xSucc, ySucc, t + Constants::dt[i], point->getG(), 0, true,point);
                }
                

                adjacentNodes.push_back(tmpPtr);
                // std::cout << "the expention of node x : " << tmpPtr->getX() << " Y : " << tmpPtr->getY() << std::endl;
            }
        }

        return adjacentNodes;
    }

    int hybridAstar::calcIndix(float x, float y, int cells_x, float t) {
        return (int(x) * cells_x + int(y)) * Constants::headings + int(t / Constants::headings);
    }

    void hybridAstar::nodeToPlan(Node3D* node, std::vector<geometry_msgs::PoseStamped>& plan) {
        Node3D* tmpPtr = node;
        geometry_msgs::PoseStamped tmpPose;
        tmpPose.header.stamp = ros::Time::now();   
        //参数后期处理，发布到RViz上进行可视化
            
        while(tmpPtr!=nullptr) {
            #ifdef TEST
            tmpPose.pose.position.x = tmpPtr->getX();
            tmpPose.pose.position.y = tmpPtr->getY();
            #else
            costmap->mapToWorld(tmpPtr->getX(), tmpPtr->getY(), tmpPose.pose.position.x, tmpPose.pose.position.y);
            #endif
            tmpPose.header.frame_id = frame_id_;
            tmpPose.pose.orientation = tf::createQuaternionMsgFromYaw(tmpPtr->getT());
            std::cout << "the goal is reverse " << tmpPtr->isReverse() << std::endl;
            plan.push_back(tmpPose);
            tmpPtr = tmpPtr->getPerd();
        }
    }

}
