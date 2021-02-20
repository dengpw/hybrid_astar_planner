/**
 * @file tf_broadcaster.cpp
 * @brief 这时实现了一个坐标系统转换的TF变换:
 *  1) odom -> map 
 *  2) map  -> path
 *  3) map  -> base_link
 */

//###################################################
//                        TF MODULE FOR THE HYBRID A*
//                        用于测试混合A*算法的一个模组
//###################################################
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// map pointer
nav_msgs::OccupancyGridPtr grid;//定义广播信息的格式

// map callback
// 这是回调函数，将当前网格图设置成新的网格图
void setMap(const nav_msgs::OccupancyGrid::Ptr map) {
  std::cout << "Creating transform for map..." << std::endl;
  grid = map;
}

void postcallback(const geometry_msgs::PoseWithCovarianceStampedPtr& post) {
  //创建tf的广播器
  static tf::TransformBroadcaster br;

  // 初始化tf数据
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(post->pose.pose.position.x, post->pose.pose.position.y, 0.0));
  tf::Quaternion q;
  //定义旋转的四元参数
  //具体的TF变换内容可以参考《机器人学导论》（美）克来格（Craig,J.J）
  q.setX(post->pose.pose.orientation.x);
  q.setY(post->pose.pose.orientation.y);
  q.setZ(post->pose.pose.orientation.z);
  q.setW(post->pose.pose.orientation.w);
  transform.setRotation(q);
  // 广播world与base_link之间的tf数据
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
  std::cout << "Transform initialpost to base_link" << std::endl;
}
int main(int argc, char** argv) {
  // initiate the broadcaster
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle n;
  ros::NodeHandle nh;
  // subscribe to map updates
  ros::Subscriber sub_map = n.subscribe("/occ_map", 1, setMap);
  ros::Subscriber sub_post = nh.subscribe("/initialpose", 1, &postcallback);
  tf::Pose tfPose;
  ros::Rate r(1);
  tf::TransformBroadcaster broadcaster;
  while (ros::ok()) {
    // transform from geometry msg to TF
    if (grid != nullptr) {
      tf::poseMsgToTF(grid->info.origin, tfPose);
    }
    // listener
    // odom to map 从odom到地图坐标系
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tfPose.getOrigin()),
        ros::Time::now(), "odom", "map"));
    // map to path 从map到路径坐标系，这里的路径坐标系是三维的坐标系
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
        ros::Time::now(), "map", "path"));
    ros::spinOnce();
    r.sleep();
  }
}
