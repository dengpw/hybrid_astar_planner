/**
   @file  test.cpp
   @brief Main entry point of the test program, the test program is using to test the hybrid_Astar plugin of
          move_base package.
   @author dengpw 2021/02/21 
*/
//函数注释！！    解决
//清理旧的标定点  解决
#include <tf/transform_listener.h>
#include "test_plugins.h"
int main(int argc, char** argv) {
    ros::init(argc,argv,"plugin_test_node");
    //ros::Duration(10)代表10秒的时间，在这里表示'tf2_ros::Buffer'实例化的对象保留10秒内的转换数据
    tf2_ros::Buffer buffer(ros::Duration(10));                                 
    //监听转换
    tf2_ros::TransformListener tf(buffer);                                      
    TestPlanner test( buffer );

    ros::spin();
    return 0;
}