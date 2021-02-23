#include "node2d.h"
#include <iostream>
namespace hybrid_astar_planner {

float Node2D::calcG(Node2D const *partent) {
    float g;
    if((abs(x - partent->x) + abs(y - partent->y)) == 2) {
        g = 1.414;
        
    }
    else g = 1;
    //std::cout << partent->g << std::endl;
    return g + partent->getG();
}

float Node2D::calcH(Node2D const &goal) {
    float dx, dy;
    dx = abs(x - goal.x);
    dy = abs(y - goal.y);
    h = dx +dy;
    // std::cout << "the h of ::" << h <<std::endl;
}



}