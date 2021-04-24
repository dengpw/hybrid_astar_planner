#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/State.h>
#include "node3d.h"
#include "node2d.h"
#include <boost/heap/binomial_heap.hpp>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
typedef ompl::base::SE2StateSpace::StateType State;
namespace hybrid_astar_planner {
// OPEN LIST AS BOOST IMPLEMENTATION
struct CompareNodes {
  // Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getF() > rhs->getF();
  }
  // Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getF() > rhs->getF();
  }
};
Node3D* dubinsShot(Node3D& start, Node3D& goal, costmap_2d::Costmap2D* costmap);
Node3D* reedsSheppShot(Node3D& start, Node3D& goal, costmap_2d::Costmap2D* costmap);
void  updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, float inspireAstar);
}

#endif //end of algorithm.h
