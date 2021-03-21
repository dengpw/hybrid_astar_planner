#include "algorithm.h"
#include "dubins.h"
#include <iostream>
#include <tf/transform_datatypes.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <costmap_2d/costmap_2d.h>
namespace hybrid_astar_planner {


    std::vector<Node2D*> gatAdjacentPoints(int cells_x, int cells_y, const unsigned char* charMap, Node2D* pathNode2D, Node2D *point ) {
        std::vector<Node2D*> adjacentNodes;
        for (int x = point->getX() - 1; x <= point->getX() + 1; ++x) {
            for (int y = point->getY() - 1; y <= point->getY() + 1; ++y) {
                if (charMap[x  + y* cells_x] < 253) {
                    pathNode2D[x * cells_y + y].setX(x);
                    pathNode2D[x * cells_y + y].setY(y);
                    pathNode2D[x * cells_y + y].setCost(charMap[x  + y* cells_x]);
                    adjacentNodes.push_back(&pathNode2D[x * cells_y + y]);
                }
            }
        }//end of for

        return adjacentNodes;
    }
    void nodeToPlan(Node2D* node, std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2D* costmap, std::string frame_id_) {
        Node2D* tmpPtr = node;
        geometry_msgs::PoseStamped tmpPose;
        tmpPose.header.stamp = ros::Time::now();   
        //参数后期处理，发布到RViz上进行可视化
        tmpPose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            std::vector<geometry_msgs::PoseStamped> replan;
        while(tmpPtr!=nullptr) {
            costmap->mapToWorld(tmpPtr->getX(), tmpPtr->getY(), tmpPose.pose.position.x, tmpPose.pose.position.y);
            tmpPose.header.frame_id = frame_id_;
            replan.push_back(tmpPose);
            tmpPtr = tmpPtr->getPerd();
        }
        int size = replan.size();
        for (int i = 0;i < size; ++i) {
            plan.push_back(replan[size - i -1 ]);
        }
    }

    float AstarInspiration(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                        std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2D* costmap, std::string frame_id_) {
        
        const unsigned char* charMap = costmap->getCharMap(); 
        int counter = 0;
        int cells_x, cells_y;
        cells_x = costmap->getSizeInCellsX();
        cells_y = costmap->getSizeInCellsY();
        float resolution = costmap->getResolution();
        boost::heap::binomial_heap<Node2D*,boost::heap::compare<CompareNodes>> openSet;
        unsigned int startx, starty, goalx, goaly;
        costmap->worldToMap(start.pose.position.x, start.pose.position.y, startx, starty);
        costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, goalx, goaly);
        Node2D* pathNode2D = new Node2D[cells_x * cells_y]();
        // std::cout << "cells" << cells_x * cells_y << std::endl;
        // std::cout << "the positon of goal pose " << goalx << " y " << goaly << std::endl;
        Node2D* startPose = &pathNode2D[ startx * cells_y + starty ];
        
        Node2D* goalPose = &pathNode2D[ goalx * cells_y + goaly ];
        
        startPose->setX( startx );//start.pose.position.x/resolution
        startPose->setY( starty );
        startPose->setG(0);
        goalPose->setX( goalx );
        goalPose->setY( goaly );
        openSet.push( startPose );
        startPose->setOpenSet();
        Node2D* tmpStart;
        while(openSet.size()) {
            ++counter;
            tmpStart = openSet.top();
            openSet.pop();
            //如果找到目标点则返回   
            if(tmpStart->getX() == goalPose->getX() && tmpStart->getY() == goalPose->getY() )
            {
              float Gvalue = tmpStart->getG();
                std::cout << "got a plan" << std::endl;
                nodeToPlan(tmpStart, plan, costmap ,frame_id_);
                std::cout << tmpStart->getG() << std::endl;
                std::cout << counter << std::endl;
                delete [] pathNode2D;
                return Gvalue;
            }
            std::vector<Node2D*> adjacentNodes = gatAdjacentPoints(cells_x, cells_y, charMap, pathNode2D, tmpStart );    
            tmpStart->setClosedSet();

            //下面正式开始A*算法的核心搜索部分
            for (std::vector<Node2D*>::iterator it = adjacentNodes.begin(); it != adjacentNodes.end(); ++it) {
                Node2D* point = *it;
                float g;
                if (!point->isClosedSet()) {
                    g = point->calcG(tmpStart);
                    if (!point->isOpenSet() || (g < point->getG())) {//
                        point->setPerd(tmpStart);
                        point->setG(g);
                        if(!point->isOpenSet()) {
                            point->calcH(goalPose);
                            point->setOpenSet();//
                            openSet.push(point);
                        }
                        else {
                            openSet.push(point);
                        }

                    }
                }
            }

        }
        
        delete [] pathNode2D;
        return 999;
    }


void  updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, costmap_2d::Costmap2D* costmap) {
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  // if (Constants::dubins) {

  //   // ONLY FOR dubinsLookup
  //   //    int uX = std::abs((int)goal.getX() - (int)start.getX());
  //   //    int uY = std::abs((int)goal.getY() - (int)start.getY());
  //   //    // if the lookup table flag is set and the vehicle is in the lookup area
  //   //    if (Constants::dubinsLookup && uX < Constants::dubinsWidth - 1 && uY < Constants::dubinsWidth - 1) {
  //   //      int X = (int)goal.getX() - (int)start.getX();
  //   //      int Y = (int)goal.getY() - (int)start.getY();
  //   //      int h0;
  //   //      int h1;

  //   //      // mirror on x axis
  //   //      if (X >= 0 && Y <= 0) {
  //   //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
  //   //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);
  //   //      }
  //   //      // mirror on y axis
  //   //      else if (X <= 0 && Y >= 0) {
  //   //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
  //   //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);

  //   //      }
  //   //      // mirror on xy axis
  //   //      else if (X <= 0 && Y <= 0) {
  //   //        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / Constants::deltaHeadingRad);
  //   //        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.getT()) / Constants::deltaHeadingRad);

  //   //      } else {
  //   //        h0 = (int)(t / Constants::deltaHeadingRad);
  //   //        h1 = (int)(goal.getT() / Constants::deltaHeadingRad);
  //   //      }

  //   //      dubinsCost = dubinsLookup[uX * Constants::dubinsWidth * Constants::headings * Constants::headings
  //   //                                + uY *  Constants::headings * Constants::headings
  //   //                                + h0 * Constants::headings
  //   //                                + h1];
  //   //    } else {

  //   /*if (Constants::dubinsShot && std::abs(start.getX() - goal.getX()) >= 10 && std::abs(start.getY() - goal.getY()) >= 10)*/
  //   //      // start
  //   //      double q0[] = { start.getX(), start.getY(), start.getT()};
  //   //      // goal
  //   //      double q1[] = { goal.getX(), goal.getY(), goal.getT()};
  //   //      DubinsPath dubinsPath;
  //   //      dubins_init(q0, q1, Constants::r, &dubinsPath);
  //   //      dubinsCost = dubins_path_length(&dubinsPath);
    
  //   //这里改用open motion planning library的算法
  //   ompl::base::DubinsStateSpace dubinsPath(Constants::r);
  //   State* dbStart = (State*)dubinsPath.allocState();
  //   State* dbEnd = (State*)dubinsPath.allocState();
  //   dbStart->setXY(start.getX(), start.getY());
  //   dbStart->setYaw(start.getT());
  //   dbEnd->setXY(goal.getX(), goal.getY());
  //   dbEnd->setYaw(goal.getT());
  //   dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  // }

  // if reversing is active use a Reeds-Shepp 
  //假如车子可以后退，则可以启动Reeds-Shepp 算法
  if (Constants::reverse && !Constants::dubins) {
    //    ros::Time t0 = ros::Time::now();
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd) * 1.2 + 0.02*start.getCost();
    // std::cout << "the cost of reed_Shepp : " << reedsSheppCost << std::endl;
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  // if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
  //   //    ros::Time t0 = ros::Time::now();
  //   // create a 2d start node
    // geometry_msgs::PoseStamped start2d;
    // geometry_msgs::PoseStamped goal2d;
    // start2d.pose.position.x = start.getX();
    // start2d.pose.position.y = start.getY();
    // goal2d.pose.position.x = goal.getX();
    // goal2d.pose.position.y = goal.getY();
    //Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    // Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    // nodes2D[(int)start.getY() * width + (int)start.getX()].setG(
      //调用A*算法，返回cost-so-far, 并在2D网格中设置相应的代价值
      // twoDCost = AstarInspiration(start2d, goal2d, costmap) ;
      // std::cout << twoDCost << std::endl;
      // );
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  // }

  // if (Constants::twoD) {
  //   // offset for same node in cell
  //   twoDoffset = sqrt( ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * 
  //                      ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
  //                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * 
  //                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY()))
  //                     );
  //   twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
  //   //getG()返回A*的启发式代价，twoDoffset为 start与goal各自相对自身所在2D网格的偏移量的欧氏距离

  // }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));//将两个代价中的最大值作为启发式值
  //注：虽然这里有三个数值，但Dubins Cost的启用和Reeds-Shepp Cost的启用是互斥的，所以实际上是计算两种cost而已
}


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