#include "node2d.h"
#include "astar.h"
namespace hybrid_astar_planner {
    bool astar::calculatePath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan ) {
        
        const unsigned char* charMap = costmap->getCharMap(); 
        int counter = 0;
        boost::heap::binomial_heap<Node2D*,boost::heap::compare<CompareNodes>> openSet;
        unsigned int startx, starty, goalx, goaly;
        costmap->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty);
        costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly);
        Node2D* pathNode2D = new Node2D[cells_x * cells_y]();
        Node2D* startPose = &pathNode2D[ startx * cells_x + starty ];
        
        Node2D* goalPose = &pathNode2D[ goalx * cells_x + goaly ];
        
        startPose->setX( startx );//start.pose.position.x/resolution
        startPose->setY( starty );
        startPose->setG(0);
        goalPose->setX( goalx );
        goalPose->setY( goaly );
        openSet.push( startPose );
        startPose->setOpenSet();
        Node2D* tmpStart;
        while(openSet.size()) {
            ++counter;
            tmpStart = openSet.top();
            openSet.pop();
            //如果找到目标点则返回   
            if(tmpStart->getX() == goalPose->getX() && tmpStart->getY() == goalPose->getY() )
            {
                std::cout << "got a plan" << std::endl;
                nodeToPlan(tmpStart,plan);
                std::cout << counter << std::endl;
                delete [] pathNode2D;
                return true;
            }
            std::vector<Node2D*> adjacentNodes = gatAdjacentPoints(cells_x, cells_y, charMap, pathNode2D, tmpStart );    
            tmpStart->setClosedSet();

            //下面正式开始A*算法的核心搜索部分
            for (std::vector<Node2D*>::iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it) {
                Node2D* point = *it;
                float g;
                if (!point->isClosedSet()) {
                    g = point->calcG(tmpStart);
                    if (!point->isOpenSet() || (g < point->getG())) {//
                        point->setPerd(tmpStart);
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

        }
        
        delete [] pathNode2D;
        return false;
    }

    std::vector<Node2D*> astar::gatAdjacentPoints(int cells_x, int cells_y, const unsigned char* charMap, Node2D* pathNode2D, Node2D *point ) {
        std::vector<Node2D*> adjacentNodes;
        for (int x = point->getX() - 1; x <= point->getX() + 1; ++x) {
            for (int y = point->getY() - 1; y <= point->getY() + 1; ++y) {
                if (charMap[x  + y* cells_y] <= 1) {
                    pathNode2D[x * cells_x + y].setX(x);
                    pathNode2D[x * cells_x + y].setY(y);
                    adjacentNodes.push_back(&pathNode2D[x * cells_x + y]);
                }
            }
        }//end of for

        return adjacentNodes;
    }

    void astar::nodeToPlan(Node2D* node, std::vector<geometry_msgs::PoseStamped>& plan) {
        Node2D* tmpPtr = node;
        geometry_msgs::PoseStamped tmpPose;
        tmpPose.header.stamp = ros::Time::now();   
        //参数后期处理，发布到RViz上进行可视化
            
        while(tmpPtr!=nullptr) {
            costmap->mapToWorld(tmpPtr->getX(), tmpPtr->getY(), tmpPose.pose.position.x, tmpPose.pose.position.y);
            tmpPose.header.frame_id = frame_id_;
            plan.push_back(tmpPose);
            tmpPtr = tmpPtr->getPerd();
        }
    }

}