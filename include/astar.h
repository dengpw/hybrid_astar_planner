#ifndef _ASTAR_H
#define _ASTAR_H

#include <vector>
#include "algorithm.h"
#include "expander.h"
#include "node2d.h"
#include <visualization_msgs/MarkerArray.h>
#include <ros/publisher.h>
namespace hybrid_astar_planner {
class Index {
    public:
        Index(int a,float b) {
            i = a;
            cost = b;
        }
        int i;
        float cost;
    };

class astar : public Expander
{

    public:
    astar(std::string frame_id, costmap_2d::Costmap2D* _costmap)
    :Expander(frame_id, _costmap) {

    }

    /**
     * @brief Find the path between the start pose and goal pose
     * @param start the reference of start pose 
     * @param goal the reference of goal pose 
     * @param cells_x the number of the cells of the costmap in x axis
     * @param cells_y the number of the cells of the costmap in y axis
     * @param plan the refrence of plan;
     * @return true if a valid plan was found.
    */
    bool calculatePath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan ,ros::Publisher& pub, visualization_msgs::MarkerArray& pathNodes );
    ~astar(){ }
    private:

    std::vector<Node2D*> gatAdjacentPoints(int cells_x, int cells_y, const unsigned char* charMap, Node2D* pathNode2D, Node2D *point );

    /**
     * @brief transform the 2Dnode to geometry_msgs::PoseStamped
     * @param node the ptr of node
     * @param plan the refrence of plan
    */
    void nodeToPlan(Node2D* node, std::vector<geometry_msgs::PoseStamped>& plan);
    /* data */
};


}//end of namespace hybrid_astar_planner

#endif //the end of astar.h