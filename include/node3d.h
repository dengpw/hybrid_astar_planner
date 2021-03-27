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
    void setCost(unsigned int C) { cost = C; }
    void setT(float _t);
    void setG(float _g) { g = _g; }
    void setH(float _h) { h = _h; }
    void setClosedSet() { c = true; o = false; }
    void setOpenSet() { o = true; }
    void setPerd(Node3D* _perd) { perd = _perd; }
    void setReverse(bool r) { reverse = r; }
    float getX(void) const { return x; }
    float getY(void) const { return y; }
    float getT(void) const { return t * Constants::deltaHeadingRad; }
    float getTheta(void) const { return theta; }
    float getF(void) const { return g + h; }
    int getCost() {return cost;}
    /**
     * @brief caculate the G value of the node
     * @return the G value of the node
     * G值为状态空间中从初始节点到当前此节点的实际代价
     *
    */
    float calcG(void);
    float getG() const { return g; }
    float calcH(Node3D const *goal);
    int getindex(int width, int depth, float resolution, unsigned int dx, unsigned int dy) { this->index = (int(x/resolution + dx) * width + int(y/resolution + dy))*depth + t; return index;}//这里的resolution变动了
    bool isOpenSet() { return o; }
    bool isClosedSet() { return c; }
    bool isReverse() { return reverse; }
    Node3D* getPerd() { return perd; }
    // RANGE CHECKING
    /// Determines whether it is appropriate to find a analytical solution.
    bool isInRange(const Node3D& goal) const;//检测是否可以分析方法找到解

    // CUSTOM OPERATORS
    /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
    bool operator == (const Node3D& rhs) const;//位置相同且朝向相似则认为是同一节点
    
    private:
        /// the x position
        float x;
        /// the y position
        float y;
        /// the t position
        int t;
        /// the cost of this node in costmap
        unsigned int cost;
        /// the theta of this node 
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
        /// the flag of node,Indicates whether the vehicle is in reverse
        bool reverse;
        /// the predecessor pointer
        Node3D* perd;
};

}//end of namespace hybrid_astar_planner

#endif //end of node3d.h