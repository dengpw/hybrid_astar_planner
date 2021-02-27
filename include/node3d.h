#ifndef _NODE3D_H
#define _NODE3D_H
#include "constants.h"
#include <cmath>
namespace hybrid_astar_planner {
class Node3D
{
public:
    Node3D(): Node3D(0, 0, 0, 999, 0, false, nullptr) {}
    Node3D(float _x , float _y, float t, float _g = 999, float _h = 0, bool _reverse = false, Node3D* _perd = nullptr):
    x(_x), y(_y), g(_g), h(_h), reverse(_reverse), perd(_perd), o(false), c(false)
    {
        setT(t);
        index = -1;
    }
    ~Node3D(){}
    
    void setX(float _x) { x = _x; }
    void setY(float _y) { y = _y; }
    void setT(float _t);
    void setG(float _g) { g = _g; }
    void setH(float _h) { h = _h; }
    void setClosedSet() { c = true; o = false; }
    void setOpenSet() { o = true; }
    void setPerd(Node3D* _perd) { perd = _perd; }
    void setReverse(bool r) { reverse = r; }
    float getX(void) { return x; }
    float getY(void) { return y; }
    float getT(void) { return t * Constants::deltaHeadingRad; }
    float getTheta(void) { return theta; }
    float getF(void) const { return g + h; }
    float calcG(Node3D const *partent);
    float getG() const { return g; }
    float calcH(Node3D const *goal);
    int getindex(int width,int depth) { this->index = (int(x) * width + int(y))*depth + t; return index;}
    bool isOpenSet() { return o; }
    bool isClosedSet() { return c; }
    bool isReverse() { return reverse; }
    Node3D* getPerd() { return perd; }
    private:
        /// the x position
        float x;
        /// the y position
        float y;
        /// the t position
        int t;
        float theta;
        /// the cost-so-far
        float g;
        /// the cost-to-go
        float h;
        /// the index of the node in the 3D array
        int index;
        /// the open value
        bool o;
        /// the closed value
        bool c;
        /// the discovered value
        bool d;

        bool reverse;
        /// the predecessor pointer
        Node3D* perd;
};


/*!
   \brief A structure to sort nodes in a heap structure
*/
// struct CompareNodes {
// /// Sorting 3D nodes by increasing C value - the total estimated cost
//   bool operator()(const Node3D* lhs, const Node3D* rhs) const {
//     return lhs->getF() > rhs->getF();
//   }
// };

}//end of namespace hybrid_astar_planner

#endif //end of node3d.h