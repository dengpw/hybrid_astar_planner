// #include "node3d.h"
#include "algorithm.h"
#include "dubins.h"
#include <iostream>
// #include <costmap_2d/costmap_2d.h>
namespace hybrid_astar_planner {


Node3D* dubinsShot(Node3D& start, Node3D& goal, costmap_2d::Costmap2D* costmap) {
  // start
  // printf("hello world!");
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::r, &path);

  unsigned int poseX, poseY;

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);
  // printf("the length of dubins %f",length);
  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  while (x <  length) {//这是跳出循环的条件之一：生成的路径没有达到所需要的长度
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(q[2]);

    // collision check
    //跳出循环的条件之二：生成的路径存在碰撞节点
    costmap->worldToMap(dubinsNodes[i].getX(), dubinsNodes[i].getY(), poseX, poseY);
    if (costmap->getCost(poseX, poseY) <= 1) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPerd(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPerd(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPerd()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    } 
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  //返回末节点，通过getPred()可以找到前一节点。
  return &dubinsNodes[i - 1];
}


}