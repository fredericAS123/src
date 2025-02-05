#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <iostream> 
#include <iomanip>  
#include "A1CtrlStates.h"
#include "A1Params.h"
#include "A1Kinematics.h"
#include "filter.hpp"
#include "Utils.h"
#include <vector>
class GazeboA1ROS {
public:
    GazeboA1ROS(ros::NodeHandle &_nh);  // 构造函数，初始化 ROS 节点句柄
    void sub_callback(const std_msgs::Float64MultiArray::ConstPtr &msg);

private:
    ros::NodeHandle nh;  // ROS 节点句柄，用于与 ROS 系统交互
    A1CtrlStates a1_ctrl_states;
    A1Kinematics a1_kin;
    double leg_offset_x[4] = {};  // 机器人四条腿的 x 偏移量
    double leg_offset_y[4] = {};  // 机器人四条腿的 y 偏移量
    double motor_offset[4] = {};  // 机器人四条腿的电机偏移量
    double upper_leg_length[4] = {};  // 机器人四条腿的上腿长度
    double lower_leg_length[4] = {};  // 机器人四条腿的下腿长度
    std::vector<Eigen::VectorXd> rho_fix_list; // 固定腿部的参数列表
    std::vector<Eigen::VectorXd> rho_opt_list; // 优化后的腿部参数列表
    // 滤波器变量
    MovingWindowFilter acc_x;  // x 轴加速度滤波器
    MovingWindowFilter acc_y;  // y 轴加速度滤波器
    MovingWindowFilter acc_z;  // z 轴加速度滤波器
    MovingWindowFilter gyro_x; // x 轴陀螺仪滤波器
    MovingWindowFilter gyro_y; // y 轴陀螺仪滤波器
    MovingWindowFilter gyro_z; // z 轴陀螺仪滤波器
    MovingWindowFilter quat_w; // 四元数 w 分量滤波器
    MovingWindowFilter quat_x; // 四元数 x 分量滤波器
    MovingWindowFilter quat_y; // 四元数 y 分量滤波器
    MovingWindowFilter quat_z; // 四元数 z 分量滤波器
};
