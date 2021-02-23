#ifndef _ALGORITHM_H
#define _ALGORITHM_H

#include <boost/heap/binomial_heap.hpp>
#include "node2d.h"
namespace hybrid_astar_planner {
// OPEN LIST AS BOOST IMPLEMENTATION
typedef boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>
        > priorityQueue;
  
}

#endif //end of algorithm.h
