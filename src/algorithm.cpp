#include "algorithm.h"
#include "dubins.h"
#include "ReedsShepp.h"
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
                                    std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2D* costmap, std::string frame_id_, bool Inspiration) {
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
            
            if(!Inspiration) {
              std::cout << "got a plan" << std::endl;
              nodeToPlan(tmpStart, plan, costmap ,frame_id_);
            }
            
            // std::cout << tmpStart->getG() << std::endl;
            // std::cout << counter << std::endl;
            delete [] pathNode2D;
            // std::cout << Gvalue/20 << std::endl;
            return Gvalue/20;
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


void  updateH(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, float inspireAstar) {

  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;

  //假如车子可以后退，则可以启动Reeds-Shepp 算法
  if (Constants::reverse && !Constants::dubins) {
    //    ros::Time t0 = ros::Time::now();

    // ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    // State* rsStart = (State*)reedsSheppPath.allocState();
    // State* rsEnd = (State*)reedsSheppPath.allocState();
    // rsStart->setXY(start.getX(), start.getY());
    // rsStart->setYaw(start.getT());
    // rsEnd->setXY(goal.getX(), goal.getY());
    // rsEnd->setYaw(goal.getT());
    // reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd) * 1 + 0.04*start.getCost();

    // std::cout << "the cost of reed_Shepp : " << reedsSheppCost << std::endl;
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
  }
    
    // ros::Time t0 = ros::Time::now();
    // geometry_msgs::PoseStamped start2d;
    // geometry_msgs::PoseStamped goal2d;
    // start2d.pose.position.x = start.getX();
    // start2d.pose.position.y = start.getY();
    // goal2d.pose.position.x = goal.getX();
    // goal2d.pose.position.y = goal.getY();
    // std::vector<geometry_msgs::PoseStamped> tmplePlan;
    // // nodes2D[(int)start.getY() * width + (int)start.getX()].setG(twoDCost = AstarInspiration(start2d, goal2d, tmplePlan, costmap, NULL, true));
    // twoDCost = AstarInspiration(start2d, goal2d, tmplePlan, costmap, "NULL", true) * 1.4  + 0.04 * start.getCost();
    // ros::Time t1 = ros::Time::now();
    // ros::Duration d(t1 - t0);

    // std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
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
  // std::cout << "the cost of reedsSheppCost : " << reedsSheppCost << "the cost of twoDCost : " << twoDCost << std::endl;
  start.setH(std::max(reedsSheppCost, inspireAstar));//将两个代价中的最大值作为启发式值
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

Node3D* reedsSheppShot(Node3D& start, Node3D& goal, costmap_2d::Costmap2D* costmap) {

  ReedsSheppStateSpace rs_planner(Constants::r);
  double length = -1;
  unsigned int poseX, poseY;
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  std::vector<std::vector<double> > rs_path;
  rs_planner.sample(q0, q1, Constants::dubinsStepSize, length, rs_path);
  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];
  int i = 0;
  for (auto &point_itr : rs_path) {
    dubinsNodes[i].setX(point_itr[0]);
    dubinsNodes[i].setY(point_itr[1]);
    dubinsNodes[i].setT(point_itr[2]);

    //collision check
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
    i++;
    } else {
    delete [] dubinsNodes;
    return nullptr;
    } 
  }
  //返回末节点，通过getPred()可以找到前一节点。
  return &dubinsNodes[i - 1];

}

}