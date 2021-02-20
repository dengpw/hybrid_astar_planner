#ifndef _PLANNERCORE_H
#define _PLANNERCORE_H
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#define POT_HIGH 1.0e10        // unassigned cell potential
#include <vector>
#include <nav_msgs/GetPlan.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>
#include <visualization_msgs/MarkerArray.h>
namespace hybrid_astar_planner {
//这个类插件实现的多态的时候，你定义了函数就必须区实现，否则会报错！！！
/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the global_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class HybridAStarPlanner : public nav_core::BaseGlobalPlanner {
    public:
        /**
         * @brief  Default constructor for the PlannerCore object
         */
        HybridAStarPlanner();
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

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        void publishPathNodes(const std::vector<geometry_msgs::PoseStamped>& path);
    protected:
        bool initialized_;
        std::string frame_id_;
        ros::Publisher plan_pub_;
        ros::Publisher path_vehicles_pub_;//用于在路径上发布车子位置
    private:

};

} //end namespace global_planner

#endif
