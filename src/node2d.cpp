#include "node2d.h"
#include <iostream>
namespace hybrid_astar_planner {

float Node2D::calcG(Node2D const *partent) {
    float g;
    if((abs(x - partent->x) + abs(y - partent->y)) == 2) {
        g = 1.4142;
        
    }
    else g = 1;
    return g + partent->getG() + 0.1*cost;
}

float Node2D::calcH(Node2D const *goal) {
    float dx, dy;
    dx = abs(x - goal->x);
    dy = abs(y - goal->y);
    h = dx + dy;
}
std::unordered_map<int, std::shared_ptr<Node2D>> GridSearch::GenerateDpMap(
      const double goal_x, const double goal_y, 
      costmap_2d::Costmap2D* costmap) {
    // printf("hello_world!!");
    const unsigned char* charMap = costmap->getCharMap(); 
    int counter = 0;
    int cells_x, cells_y;
    cells_x = costmap->getSizeInCellsX();
    cells_y = costmap->getSizeInCellsY();
    float resolution = costmap->getResolution();
    std::priority_queue<std::pair<int, double>,
                        std::vector<std::pair<int, double>>, cmp> 
            open_pq;
    std::unordered_map<int, std::shared_ptr<Node2D>> open_set;
    std::unordered_map<int, std::shared_ptr<Node2D>> dp_map_;
    std::shared_ptr<Node2D> goal_node = std::make_shared<Node2D>(goal_x, goal_y);

    // dp_map_ = decltype(dp_map_)();
    goal_node->setX( goal_x );
    goal_node->setY( goal_y );
    goal_node->setG(0);
    // std::cout << "hahahahah"<< " x: " << goal_x << " y: "<< goal_y <<std::endl;
    // printf("explored node num is : %d",counter);
    // std::cout << "hahahha" << std::endl;
    // dp_map_.emplace(goal_node->getindex(cells_x), goal_node);
    open_set.emplace(goal_node->getindex(cells_x), goal_node );
    // open_set.emplace(node1->getindex(cells_x), node1 );
    // open_set.emplace(node2->getindex(cells_x), node2 );
    // open_set.emplace(node3->getindex(cells_x), node3 );
    
    open_pq.emplace(goal_node->getindex(cells_x), goal_node->getG());
    // open_pq.emplace(node1->getindex(cells_x), node1->getCost());
    // open_pq.emplace(node2->getindex(cells_x), node2->getCost());
    // open_pq.emplace(node3->getindex(cells_x), node3->getCost());
    // if (open_set.find(node2->getindex(cells_x)) != open_set.end()) {
    //     std::cout << "node is in the open set" << std::endl;
    // } else {
    //     std::cout << "error node is valid" << std::endl;
    // }
    while(!open_pq.empty()) {
        // printf("explored node num is : %d",counter);
        ++counter;
        int id = open_pq.top().first;
        open_pq.pop();
        std::shared_ptr<Node2D> current_node = open_set[id];
        // std::cout << "hahahahah" <<current_node->getG() << " x: " <<current_node->getX() << " y: "<< current_node->getY() <<std::endl;
        dp_map_.emplace(current_node->getindex(cells_x), current_node);

        // std::cout << "the cost of node" << current_node->getG() << std::endl;
        std::vector<std::shared_ptr<Node2D>> adjacent_nodes = 
            getAdjacentPoints(cells_x, cells_y, charMap, current_node );
        // printf("explored node num is : %d",counter);
        // // //下面正式开始A*算法的核心搜索部分
        // std::cout << "hahahha" << std::endl;
        for (std::vector<std::shared_ptr<Node2D>>::iterator 
                it = adjacent_nodes.begin(); it != adjacent_nodes.end(); ++it) {
            std::shared_ptr<Node2D> next_node = *it;
            // std::cout << "bbbbbbbbbb" <<next_node->getG() << " x: " <<next_node->getX() << " y: "<< next_node->getY() <<std::endl;
            if (dp_map_.find(next_node->getindex(cells_x)) != dp_map_.end()) {
                continue;
            }
            // std::cout << "lalalla" << std::endl;
            if (open_set.find(next_node->getindex(cells_x)) != open_set.end()) {
                // std::cout << "sssss" << std::endl;
                if (open_set[next_node->getindex(cells_x)]->getG() > next_node->getG()) {
                    open_set[next_node->getindex(cells_x)]->setCost(next_node->getG());
                    open_set[next_node->getindex(cells_x)]->setPerd_(current_node);
                }

            } else {
                ++counter;
                next_node->setPerd_(current_node);
                open_set.emplace(next_node->getindex(cells_x), next_node );
                open_pq.emplace(next_node->getindex(cells_x), next_node->getG());
            }
        }
        // std::cout << counter << std::endl;
    }
    
    printf("explored node num is : %d \n",counter);
    return dp_map_;
}

std::vector<std::shared_ptr<Node2D>> GridSearch::getAdjacentPoints(int cells_x,
    int cells_y, const unsigned char* charMap, std::shared_ptr<Node2D> point) {
    std::vector<std::shared_ptr<Node2D>> adjacentNodes;
    float g = 0;
    // std::cout << "the cost-so-far of this node" << point->getG() << std::endl;
    for (int x = point->getX() - 1; x <= point->getX() + 1; ++x) {
        for (int y = point->getY() - 1; y <= point->getY() + 1; ++y) {
            if (charMap[x  + y* cells_x] < 250 && 
                charMap[x  + y* cells_x] >= 0) {
                std::shared_ptr<Node2D> node = std::make_shared<Node2D>(x,y);
                if((abs(x - point->getX()) + abs(y - point->getY())) == 2) {
                    g = 1.4142;
                } else {
                    g = 1;
                }
                node->SetPathCost(point->getG() + 0.1*charMap[x  + y* cells_x] + g);
                node->setX(x);
                node->setY(y);
                adjacentNodes.emplace_back(node);
            }
        }
    }//end of for

    return adjacentNodes;
}

}//end of namespace hybrid_astar_planner
