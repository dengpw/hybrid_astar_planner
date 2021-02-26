#include "node3d.h"
#include "hybrid_astar.h"
namespace hybrid_astar_planner {
    bool hybridAstar::calculatePath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan ) {
        boost::heap::binomial_heap<Node3D*,boost::heap::compare<CompareNodes>> openset;
        const unsigned char* charMap = costmap->getCharMap(); 
        Node3D* pathNode3D = new Node3D[cells_x * cells_y]();
        delete [] pathNode3D;
        std::cout << "hello world!\n This is hybrid_astar_planner!" << std::endl;
        return false;
    }

    std::vector<Node3D*> hybridAstar::gatAdjacentPoints(int cells_x, int cells_y, const unsigned char* charMap, Node3D* pathNode3D, Node3D *point ) {
        std::vector<Node3D*> adjacentNodes;
        for (int x = point->getX() - 1; x <= point->getX() + 1; ++x) {
            for (int y = point->getY() - 1; y <= point->getY() + 1; ++y) {
                if (charMap[x  + y* cells_y] <= 1) {
                    pathNode3D[x * cells_x + y].setX(x);
                    pathNode3D[x * cells_x + y].setY(y);
                    adjacentNodes.push_back(&pathNode3D[x * cells_x + y]);
                }
            }
        }//end of for

        return adjacentNodes;
    }

    void hybridAstar::nodeToPlan(Node3D* node, std::vector<geometry_msgs::PoseStamped>& plan) {
        Node3D* tmpPtr = node;
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
