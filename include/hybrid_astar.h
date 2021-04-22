#ifndef _HYBRID_ASTAR_H
#define _HYBRID_ASTAR_H

#include <vector>
#include "algorithm.h"
#include "expander.h"
#include "node3d.h"
#include <visualization_msgs/MarkerArray.h>
#include <ros/publisher.h>
// #define TEST
#define point_accuracy 0.5
#define theta_accuracy 2
namespace hybrid_astar_planner {

class hybridAstar : public Expander
{

    public:
    /**
     * @brief  Default constructor for the HybridAStarPlanner object
    */
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
    bool calculatePath(
        const geometry_msgs::PoseStamped& start, 
        const geometry_msgs::PoseStamped& goal,
        int cellsX, int cellsY, std::vector<geometry_msgs::PoseStamped>& plan,
        ros::Publisher& pub, visualization_msgs::MarkerArray& pathNodes);
    
    /**
     * @brief Default deconstructor for the HybridAStarPlanner object
    */
    ~hybridAstar(){ }

    private:
 
    /**
     * @brief Get the adjacent pose of a given pose
     * @param cells_x the number of the cells of the costmap in x axis
     * @param cells_y the number of the cells of the costmap in y axis
     * @param charMap 
    */
    std::vector<Node3D*> gatAdjacentPoints(int dir, int cells_x, int cells_y, const unsigned char* charMap, Node3D *point);

    /**
     * @brief judge whether is reach the goal pose
     * @param node the refrence of the node
     * @param goalPose the goal of the planner
     * @return true if reach the goal
    */
    bool reachGoal(Node3D* node, Node3D* goalPose); 

    /**
     * @brief get the index of node
     * @param x the x position axis of the node
     * @param y the y position axis of the node
     * @param cells_x the scale of cells in x axis
     * @param t the depth of the 3D nodes
     * @return the index of node 
    */
    int calcIndix(float x, float y, int cells_x, float t); 

    /**
     * @brief transform the 2Dnode to geometry_msgs::PoseStamped
     * @param node the ptr of node
     * @param plan the refrence of plan
    */
    void nodeToPlan(Node3D* node, std::vector<geometry_msgs::PoseStamped>& plan);
    std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;
};




}//end of namespace hybrid_astar_planner

#endif //the end of astar.h