#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
/*********************************************************************
 *
 *  BSD 3-Clause License
 *
 *  Copyright (c) 2021, dengpw
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1 Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   2 Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   3 Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
// #define POT_HIGH 1.0e10        // unassigned cell potential
#include <vector>
#include <nav_msgs/GetPlan.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>
#include <visualization_msgs/MarkerArray.h>
#include "node2d.h"
#include "node3d.h"
namespace hybrid_astar_planner {
//类插件实现的多态的时候，若在头文件中定义了函数，那么就必须有这个函数的实现，否则会报错！！！
/**
 * @class HybridAStarPlanner
 * @brief Provides a ROS wrapper for the HybridAStarPlanner planner which runs a fast, interpolated navigation function on a costmap.
 */

class HybridAStarPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the HybridAStarPlanner object
         */
        HybridAStarPlanner();

        /**
         * @brief  Default deconstructor for the HybridAStarPlanner object
         */
        ~HybridAStarPlanner();

        /**
         * @brief  Initialization function for the HybridAStarPlanner
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning.And the costmap is get from the topic name /map
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);
   
        /**
         * @brief Publish the plan to RVIZ 
         * @param path the vector contain the path
         */
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        /**
         * @brief Publish the path node to RVIZ 
         * @param path the vector contain the path
         */
        void publishPathNodes(const std::vector<geometry_msgs::PoseStamped>& path);

        /**
         * @brief The call back function of makeplan service
         * @param req 
         * @param resp the plan the planner made
         * @return True if a valid plan was found, false otherwise
        */
        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
    protected:

        bool initialized_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        ros::Publisher path_vehicles_pub_;//用于在路径上发布车子位置
        costmap_2d::Costmap2D* costmap;
        
    private:
        /**
         * @brief Clarn the visualization_msgs::Marker 清理可视化信息的标记点
         */
        void clearPathNodes(void);



        /**
         * @brief Check whethe the start pose is available
         * @param start A reference to start pose
         * @return True if the start pose is available
        */
        bool checkStartPose(const geometry_msgs::PoseStamped &start);

        /**
         * @brief Check whethe the goal pose is available
         * @param goal A reference to goal pose
         * @return True if the goal pose is available
        */
        bool checkgoalPose(const geometry_msgs::PoseStamped &goal);
        visualization_msgs::MarkerArray pathNodes;//节点数据结构，用于可视化
        double resolution;
        ros::ServiceServer make_plan_srv_;
        bool use_hybrid_astar;

};


} //end namespace hybrid_astar_planner

#endif
