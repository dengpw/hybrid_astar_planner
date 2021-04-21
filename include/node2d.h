#ifndef _NODE2D_H
#define _NODE2D_H

#include <cmath>
#include <unordered_map>
#include <costmap_2d/costmap_2d.h>
namespace hybrid_astar_planner {

class Node2D
{
public:
    Node2D(): Node2D(0, 0, 999, 0, nullptr) {}
    Node2D(int _x , int _y, float _g = 999, float _h = 0, Node2D* _perd = nullptr):
    x(_x), y(_y), g(_g), h(_h), perd(_perd), o(false), c(false)
    {
        index = -1;
    }
    ~Node2D(){}
    
    void setX(int _x) { x = _x; }
    void setY(int _y) { y = _y; }
    void setCost(unsigned int C) { cost = C; }
    void setG(float _g) { g = _g; }
    void setH(float _h) { h = _h; }
    void setClosedSet() { c = true; o = false; }
    void setOpenSet() { o = true; }
    void setPerd(Node2D* _perd) { perd = _perd; }
    void setPerd_(std::shared_ptr<Node2D> _perd) { perd_node = _perd; }
    int getX(void) { return x; }
    int getY(void) { return y; }
    float getF(void) const { return g + h; }
    float calcG(Node2D const *partent);
    float getG() const { return g; }
    float calcH(Node2D const *goal);
    int getindex(int width) { return (y * width + x); }
    int getCost() {return cost;}
    bool isOpenSet() { return o; }
    bool isClosedSet() { return c; }
    Node2D* getPerd() { return perd; }
    void SetPathCost(const float path_cost) {
        g = path_cost;
    }
    private:
        // float cost_ = 999;
        /// the x position
        int x;
        /// the y position
        int y;
        /// the cost of current node
        unsigned int cost;
        /// the cost-so-far
        float g;
        /// the cost-to-go
        float h;
        /// the index of the node in the 2D array
        int index;
        /// the open value
        bool o;
        /// the closed value
        bool c;
        /// the discovered value
        bool d;
        /// the predecessor pointer
        Node2D* perd;
        std::shared_ptr<Node2D> perd_node = nullptr;
};

class GridSearch {
    public:
    GridSearch()
    {

    }
    ~GridSearch()
    {
        
    }
    std::vector<std::shared_ptr<Node2D>> getAdjacentPoints(int cells_x,
        int cells_y, const unsigned char* charMap, std::shared_ptr<Node2D> point);

    std::unordered_map<int, std::shared_ptr<Node2D>> GenerateDpMap(
        const double goal_x, const double goal_y, 
        costmap_2d::Costmap2D* costmap);
    // std::unordered_map<int, std::shared_ptr<Node2D>> dp_map_;
    private:

    // std::unordered_map<int, std::shared_ptr<Node2D>> dp_map_;
    struct cmp {
        // Sorting 3D nodes by increasing C value - the total estimated cost
        bool operator()(const std::pair<int, double>& left,
                    const std::pair<int, double>& right) const {
            return left.second >= right.second;
        }
    };
};

}//end of namespace hybrid_astar_planner

#endif //end of node2d.h