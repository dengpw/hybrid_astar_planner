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
                // std::cout << "the t of this node: " << t << " the t of pred node: " << perd->t << std::endl;
                if (t == perd->t) {
                    g = Constants::dx[0];
                    
                }
                else {
                    g = Constants::dx[0] * Constants::penaltyTurning;// 
                    // printf("hello \n");
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



    //###################################################
    //                                        IS IN RANGE
    //###################################################
    bool Node3D::isInRange(const Node3D& goal) const {
    int random = rand() % 10 + 1;//产生位于[1, 10]的随机数
    float dx = std::abs(x - goal.x) / random;
    float dy = std::abs(y - goal.y) / random;
    return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;//距离的平方和在100以内则认为可达
    }

    //###################################################
    //                                 3D NODE COMPARISON
    //###################################################
    //3d节点的比较函数：x和y同时相同、并且theta在一个阈值范围内时可以认为是同一个Node
    bool Node3D::operator == (const Node3D& rhs) const {
    return (int)x == (int)rhs.x &&
            (int)y == (int)rhs.y &&
            (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
            std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
    }
}