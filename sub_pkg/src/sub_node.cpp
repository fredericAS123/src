#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <iostream> // 引入输入输出流库
#include <iomanip>  // 用于设置输出格式
#include "A1CtrlStates.h"
#include "A1Params.h"
#include "Utils.h"
#include "sub_node.h"

int main(int argc, char *argv[])
{
    
    ros::init(argc,argv,"sub_node");
    ros::NodeHandle nh;
    // GazeboA1ROS gazebo_a1_ros(nh);
    // ros::Subscriber sub = nh.subscribe("simulation_data", 10, &GazeboA1ROS::sub_callback, &gazebo_a1_ros);
    std::unique_ptr<GazeboA1ROS> gazebo_a1_ros = std::make_unique<GazeboA1ROS>(nh);
    while(ros::ok())
    {
        ros::spinOnce();
    } 
    //  启动ROS异步执行器，允许多个线程同时执行
    // ros::AsyncSpinner spinner(12);  // 创建一个多线程的 ROS 异步执行器，最多支持12个线程
    // spinner.start();  // 启动执行器
    // ros::waitForShutdown();  // 等待节点关闭
    return 0;
}
