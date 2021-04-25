# hybrid_astar_planner

This is a global planner plugin of ROS move_base package.

## Introduction
Traditional robot navigation algorithms (such as the logistics storage robot used by Jingdong or Amazon)  
mostly use a * algorithm or Dijkstra search algorithm to planning its trajectory after the map is gridded.   
Most of them are differential models. Obviously, this kind of path is still difficult to realize for traditional  
four-wheel vehicles whitch has nonholonomic constraint (vehicles with Ackerman steering mechanism).  
So I designed a hybrid a * plug-in for ROS as my graduation project

## How to Use

### <a name="dependencies"></a>Dependencies
Install all dependencies
* [Open Motion Planning Library (OMPL)](http://ompl.kavrakilab.org/)
* [ros_map_server](http://wiki.ros.org/map_server)

### <a name="setup"></a><font color=Darkorange size=5 >Setup</font>

Run the following command to clone, build, and launch the package (requires a sources ROS environment):

```
sudo apt install libompl-dev \
&& mkdir -p ~/catkin_ws/src \
&& cd ~/catkin_ws/src \
&& git clone https://github.com/dengpw/hybrid_astar_planner.git  \
&& cd .. \
&& catkin_make \
&& source devel/setup.bash \
&& rospack profile \
```

You can run the folling command to test whether the plug-in is install propely
```
&& roslaunch hybrid_astar_planner test.launch
```
It can run a static teajectory planner using hybrid A*   
If everything run fine,you can use this plugin in ROS naviagation package by making a special declaration before starting the launch file:  
    &lt; `param name="base_global_planner" value="global_planner/GlobalPlanner"` / &gt;  
    &lt; `param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS"` / &gt;