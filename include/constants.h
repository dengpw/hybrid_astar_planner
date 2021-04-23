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
static const double width = 0.18 + 2 * bloating;//车的宽度

/// [m] --- The length of the vehicle
static const double length = 0.22 + 2 * bloating;//车的长度

/*
* 车模需要转弯半径为0.75米的
* 车身长度需要0.15m(长) * 0.16m(轮宽)
*/
/// [m] --- the Minimum turning radius 车辆最小转弯半径 
static const double r = 1;

/// [m] --- The number of discretizations in heading
/// 车体朝向的离散数量
// static const int headings = 72;
static const int headings = 72;
// const float dy[] = { 0,        -0.0415893,  0.0415893};
// const float dx[] = { 0.7068582,   0.705224,   0.705224};
// const float dt[] = { 0,         0.1178097,   -0.1178097};

const float dy[] = { 0,        -0.005198,  0.005198};
const float dx[] = { 0.0883573,   0.088153,   0.088153};
const float dt[] = { 0,         0.1178097,   -0.1178097};
/// [°] --- The discretization value of the heading (goal condition)
/// 朝向离散度数(以度表示)
static const float deltaHeadingDeg = 360 / (float)headings; 

/// [c*M_PI] --- The discretization value of heading (goal condition)
static const float deltaHeadingRad = 2 * M_PI / (float)headings; //朝向离散步长(以弧度表示)

/// [c*M_PI] --- The heading part of the goal condition 
static const float deltaHeadingNegRad = 2 * M_PI - deltaHeadingRad;

/// A flag to toggle the connection of the path via Dubin's shot (true = on; false = off)
static const bool dubinsShot = false; //切换Dubin路径的开关

/// A flag to toggle the connection of the path via reedsSheppShot (true = on; false = off)
static const bool reedsSheppShot = true; //切换Dubin路径的开关

/// A flag to toggle the Dubin's heuristic, this should be false, if reversing is enabled (true = on; false = off)
static const bool dubins = false;//Dubin路径的切换开关: 若车子可以倒退，值为false
// ___________________
// HEURISTIC CONSTANTS

/// [#] --- A factor to ensure admissibility of the holonomic with obstacles heuristic
static const float factor2D = sqrt(5) / sqrt(2) + 1;
/// [#] --- A movement cost penalty for turning (choosing non straight motion primitives)
static const float penaltyTurning = 1.05;
/// [#] --- A movement cost penalty for reversing (choosing motion primitives > 2)
static const float penaltyReversing = 1.5;
/// [#] --- A movement cost penalty for change of direction (changing from primitives < 3 to primitives > 2)
static const float penaltyCOD = 1.5;
/// [m] --- The distance to the goal when the analytical solution (Dubin's shot) first triggers
static const float dubinsShotDistance = 100;
/// [m] --- The step size for the analytical solution (Dubin's shot) primarily relevant for collision checking
static const float dubinsStepSize = 0.088;

}


}

#endif