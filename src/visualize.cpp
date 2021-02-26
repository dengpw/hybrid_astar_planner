#include "planner_core.h"

namespace hybrid_astar_planner {
    

    void HybridAStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }
        //create a message for the plan
        geometry_msgs::PoseStamped transform_path;
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        gui_path.header.frame_id = frame_id_;
        gui_path.header.stamp = ros::Time::now();

        // Extract the plan in world co-ordinates, we assume the path is all in the same frame
        for (unsigned int i = 0; i < path.size(); i++) {
            transform_path.pose.position = path[i].pose.position;
            gui_path.poses[i] = transform_path;//
        }

        plan_pub_.publish(gui_path);
        std::cout << "Publish the path to Rviz" << std::endl;
    }//end of publishPlan


    void HybridAStarPlanner::publishPathNodes(const std::vector<geometry_msgs::PoseStamped>& path) {
        if (!initialized_) {
            ROS_ERROR(
                    "This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return;
        }
        visualization_msgs::Marker pathVehicle;
        int nodeSize = path.size();
        pathVehicle.header.stamp = ros::Time(0);
        pathVehicle.color.r = 102.f / 255.f;
        pathVehicle.color.g = 217.f / 255.f;
        pathVehicle.color.b = 239.f / 255.f;
        pathVehicle.type = visualization_msgs::Marker::ARROW;
        pathVehicle.header.frame_id = frame_id_;
        pathVehicle.scale.x = 1;
        pathVehicle.scale.y = 1;
        pathVehicle.scale.z = 1;
        pathVehicle.color.a = 0.1;
        // 转化节点，并同时加上时间戳等信息
        for(int i = 0; i<nodeSize; i++) {
            pathVehicle.header.stamp = ros::Time(0);
            pathVehicle.pose = path[i].pose;
            pathVehicle.id = i;
            pathNodes.markers.push_back(pathVehicle);
        }
        // 发布这些车辆位置标记点
        path_vehicles_pub_.publish(pathNodes);
        
    }//end of publishPathNodes

    void HybridAStarPlanner::clearPathNodes() {
        // 初始化并配置节点为全清空模式
        visualization_msgs::Marker node;
        pathNodes.markers.clear();
        node.action = visualization_msgs::Marker::DELETEALL;
        node.header.frame_id = frame_id_;
        node.header.stamp = ros::Time(0);
        node.id = 0;
        node.action = 3;
        pathNodes.markers.push_back(node);
        path_vehicles_pub_.publish(pathNodes);

        std::cout << "clean the path nodes" <<std::endl;
    }
    

}
