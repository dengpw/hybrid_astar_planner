#ifndef _EXPANDER_H
#define _EXPANDER_H
#include <vector>
#include <geometry_msgs/Point.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/heap/binomial_heap.hpp>
#include <ros/publisher.h>
#include <visualization_msgs/MarkerArray.h>
namespace hybrid_astar_planner {

/**
 * @brief the base class of astar or hybrid astar planner
*/
class Expander
{

    public:
    Expander(std::string frame_id, costmap_2d::Costmap2D* _costmap)
    :frame_id_(frame_id), costmap(_costmap) {

    }

    virtual bool calculatePath(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        int cells_x, int cells_y, std::vector<geometry_msgs::PoseStamped>& plan ,ros::Publisher& pub, visualization_msgs::MarkerArray& pathNodes) = 0;
    virtual ~Expander() {}
    protected:
    std::string frame_id_;
    costmap_2d::Costmap2D* costmap;
    /* data */
};


} //end of namespace hybrid_astar_planner
#endif //end of expander.h