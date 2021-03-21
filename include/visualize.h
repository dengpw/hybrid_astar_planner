#ifndef _VISUALIZE_H
#define _VISUALIZE_H
#include "node3d.h"
#include <visualization_msgs/MarkerArray.h>
#include <ros/publisher.h>
namespace hybrid_astar_planner {
    void publishSearchNodes(Node3D node,ros::Publisher& pub, visualization_msgs::MarkerArray& pathNodes, int i);
}

#endif//end of visualize,h