#ifndef _ASTAR_H
#define _ASTAR_H

#include <vector>
#include <algorithm>
#include "expander.h"
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
    bool calculatePath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan );
    ~astar(){ }
    private:

    /**
     * @brief transform the 2Dnode to geometry_msgs::PoseStamped
     * @param node the ptr of node
     * @param plan the refrence of plan
    */
    void nodeToPlan(Node2D* node, std::vector<geometry_msgs::PoseStamped>& plan);
    /* data */
};

typedef boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>
        > priorityQueue;


}//end of namespace hybrid_astar_planner

#endif //the end of astar.h