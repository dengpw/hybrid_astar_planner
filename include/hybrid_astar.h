#ifndef _HYBRID_ASTAR_H
#define _HYBRID_ASTAR_H

#include <vector>
#include "algorithm.h"
#include "expander.h"
#include "node3d.h"
#define TEST
namespace hybrid_astar_planner {

class hybridAstar : public Expander
{

    public:
    hybridAstar(std::string frame_id, costmap_2d::Costmap2D* _costmap)
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
                                        int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan );
    ~hybridAstar(){ }
    private:
 
    /**
     * @brief Get the adjacent pose of a given pose
     * @param cells_x the number of the cells of the costmap in x axis
     * @param cells_y the number of the cells of the costmap in y axis
     * @param charMap 
    */
    std::vector<Node3D*> gatAdjacentPoints(int dir, int cells_x, int cells_y, const unsigned char* charMap, Node3D* pathNode3D, Node3D *point );

    int calcIndix(float x, float y, int cells_x, float t); 

    /**
     * @brief transform the 2Dnode to geometry_msgs::PoseStamped
     * @param node the ptr of node
     * @param plan the refrence of plan
    */
    void nodeToPlan(Node3D* node, std::vector<geometry_msgs::PoseStamped>& plan);
    /* data */
};




}//end of namespace hybrid_astar_planner

#endif //the end of astar.h