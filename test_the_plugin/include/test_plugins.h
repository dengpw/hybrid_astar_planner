#ifndef _TEST_PLUGINS
#define _TEST_PLUGINS
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_core/base_global_planner.h>
#include <tf2_ros/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <ros/node_handle.h>
class TestPlanner{
    public:
        /**
         * @brief   Default constructor for the TestPlanner
         * @param tf A reference to a TransformListener
         */
        TestPlanner(tf2_ros::Buffer &tf);
        /**
         * @brief   Destructor - Cleans up 
         **/
        ~TestPlanner();
        /**
         * @brief   Call back function of a suscriber 目的是接收RViz传过来的goal pose
         * @param  _goal A reference to geometry_msgs::PoseStamped::ConstPtr
         */
        void setgoal(const geometry_msgs::PoseStamped::ConstPtr& _goal);

    private:
        /**
         * @brief   Transform the Start pose from tf tree 将起始点从TF转化树中提取出来(测试程序将bese_link作为起始点提取)
         * 
         */
        bool transformStarPose(void);
        //智能指针，用来加载类插件，保存类插件的地址。这里没有使用标准库是因为Classloder的createInstance函数支持boost库，并没有使用标准库
        boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
        //路径规划的vector类模板，用来保存路径
        std::vector<geometry_msgs::PoseStamped>* planner_plan_;
        geometry_msgs::PoseStamped goal_pose;
        geometry_msgs::PoseStamped start_pose;
        geometry_msgs::TransformStamped start_transform;
        ros::NodeHandle n;
        costmap_2d::Costmap2DROS* costmap;
        geometry_msgs::PoseStamped robot_pose;
        std::string global_planner;
        tf2_ros::Buffer& tf;
        ros::Subscriber make_plane;

};

#endif //!_TEST_PLUGINS