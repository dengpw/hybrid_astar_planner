#include "node3d.h"
#include <iostream>
#include <math.h>
// 设计启发函数
namespace hybrid_astar_planner {
    void Node3D::setT(float _t) {
        if( _t <0 ) {
            t = 72 + _t / Constants::deltaHeadingRad;
        }
        else {
            t = _t / Constants::deltaHeadingRad; 
        }
        
    }
    
    float Node3D::calcG() {
        float g;

        if(reverse) {
            if (reverse != perd->reverse) {//对方向改变进行的惩罚
                g = Constants::dx[0] * Constants::penaltyCOD * Constants::penaltyReversing;
            }
            else {
                g = Constants::dx[0] * Constants::penaltyReversing; 
            }
        }
        else {
            if (reverse != perd->reverse) {//对方向改变进行的惩罚
                g = Constants::dx[0] * Constants::penaltyCOD;
            }
            else {
                if (t == perd->t) {
                    g = Constants::dx[0]; 
                }
                else {
                    g = Constants::dx[0] * Constants::penaltyTurning; 
                }
                
            }
        }
        return g + perd->getG();
    }

    float Node3D::calcH(Node3D const *goal) {
        float dx, dy;
        dx = fabs(x - goal->x);
        dy = fabs(y - goal->y);
        h = dx +dy;
    }
}