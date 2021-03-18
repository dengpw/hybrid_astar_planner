#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include "node3d.h"
#include "node2d.h"
#include "collisiondetection.h"
#include <boost/heap/binomial_heap.hpp>
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
Node3D* dubinsShot(Node3D& start, Node3D& goal);

}

#endif //end of algorithm.h
