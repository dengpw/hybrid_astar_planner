#ifndef _ASTAR_H
#define _ASTAR_H

#include <vector>
#include <algorithm>

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
    

}//end of namespace hybrid_astar_planner

#endif //the end of astar.h