#ifndef _NODE3D_H
#define _NODE3D_H

#include <cmath>
namespace hybrid_astar_planner {
class Node3D
{
public:
    Node3D(): Node3D(0, 0, 999, 0, nullptr) {}
    Node3D(int _x , int _y, float _g = 999, float _h = 0, Node3D* _perd = nullptr):
    x(_x), y(_y), g(_g), h(_h), perd(_perd), o(false), c(false)
    {
        index = -1;
    }
    ~Node3D(){}
    
    void setX(int _x) { x = _x; }
    void setY(int _y) { y = _y; }
    void setG(float _g) { g = _g; }
    void setH(float _h) { h = _h; }
    void setClosedSet() { c = true; o = false; }
    void setOpenSet() { o = true; }
    void setPerd(Node3D* _perd) { perd = _perd; }
    int getX(void) { return x; }
    int getY(void) { return y; }
    float getF(void) const { return g + h; }
    float calcG(Node3D const *partent);
    float getG() const { return g; }
    float calcH(Node3D const *goal);
    int getindex(int width) { return (y * width + x); }
    bool isOpenSet() { return o; }
    bool isClosedSet() { return c; }
    Node3D* getPerd() { return perd; }
    private:
        /// the x position
        int x;
        /// the y position
        int y;
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