#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include "node3d.h"
#include "node2d.h"
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
// struct CompareNodes3D {
//   // Sorting 3D nodes by increasing C value - the total estimated cost
//   bool operator()(const Node3D* lhs, const Node3D* rhs) const {
//     return lhs->getF() > rhs->getF();
//   }
//   // Sorting 2D nodes by increasing C value - the total estimated cost
// //   bool operator()(const Node2D* lhs, const Node2D* rhs) const {
// //     return lhs->getF() > rhs->getF();
// //   }
// };
// typedef boost::heap::binomial_heap<Node2D*,boost::heap::compare<CompareNodes>> priorityQueue;  
}

#endif //end of algorithm.h
