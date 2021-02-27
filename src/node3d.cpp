#include "node3d.h"

namespace hybrid_astar_planner {
    void Node3D::setT(float _t) {
        if( _t <0 ) {
            t = 72 + _t / Constants::deltaHeadingRad;
        }
        else {
            t = _t / Constants::deltaHeadingRad; 
        }
        
    }
    float Node3D::calcG(Node3D const *partent) {
        float g;
        // if((abs(x - partent->x) + abs(y - partent->y)) == 2) {
        //     g = 1.414;
            
        // }
        // else g = 1;
        if(reverse) {
            g = 3;
        }
        else {
            g = 1;
        }
        return g + partent->getG();
    }
    float Node3D::calcH(Node3D const *goal) {
        float dx, dy;
        dx = abs(x - goal->x);
        dy = abs(y - goal->y);
        h = dx +dy;
    }
}