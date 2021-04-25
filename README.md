# hybrid_astar_planner

This is a global planner plugin of ROS move_base package.

## Introduction
Traditional robot navigation algorithms (such as the logistics storage robot used by Jingdong or Amazon)  
mostly use a * algorithm or Dijkstra search algorithm to planning its trajectory after the map is gridded.   
Most of them are differential models. Obviously, this kind of path is still difficult to realize for traditional  
four-wheel vehicles whitch has nonholonomic constraint (vehicles with Ackerman steering mechanism).  
So I designed a hybrid a * plug-in for ROS as my graduation project

## How to Use

1.You should dowanload the soft