#ifndef _CONSTANTS
#define _CONSTANTS

#include <cmath>

namespace hybrid_astar_planner {

namespace Constants {
/// A flag to toggle reversing (true = on; false = off)
/// 设置是否允许车辆后退的标志位 true表示可以倒退；false表示只能前进不能倒退   
static const bool reverse = true;  

/// [#] --- Limits the maximum search depth of the algorithm, possibly terminating without the solution
/// 最大迭代次数
static const int iterations = 30000; 

/// [m] --- Uniformly adds a padding around the vehicle
/// 膨胀范围
static const double bloating = 0; 

/// [m] --- The width of the vehicle
static const double width = 1.75 + 2 * bloating;//车的宽度

/// [m] --- The length of the vehicle
static const double length = 2.65 + 2 * bloating;//车的长度

/// [m] --- The number of discretizations in heading
/// 车体朝向的离散数量
static const int headings = 72;

const float dy[] = { 0,        -0.0415893,  0.0415893};
const float dx[] = { 0.7068582,   0.705224,   0.705224};
const float dt[] = { 0,         0.1178097,   -0.1178097};

/// [°] --- The discretization value of the heading (goal condition)
/// 朝向离散度数(以度表示)
static const float deltaHeadingDeg = 360 / (float)headings; 

/// [c*M_PI] --- The discretization value of heading (goal condition)
static const float deltaHeadingRad = 2 * M_PI / (float)headings; //朝向离散步长(以弧度表示)

/// [c*M_PI] --- The heading part of the goal condition 
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;
}


}

#endif