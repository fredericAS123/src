#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <iostream> // 引入输入输出流库
#include <iomanip>  // 用于设置输出格式
#include "A1CtrlStates.h"
#include "A1Params.h"
#include "Utils.h"
#include "sub_node.h"
void GazeboA1ROS::sub_callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // 数据长度校验
    constexpr size_t EXPECTED_SIZE = 62 ;
    if (msg->data.size() != EXPECTED_SIZE)
    {
        ROS_ERROR_THROTTLE(1.0, "Invalid simulation data size! Expected %zu, got %zu",
                            EXPECTED_SIZE, msg->data.size());
        return;
    }

    // ======================= 位姿信息 =======================
    // 四元数（msg.data[40-43]: x, y, z, w）
    a1_ctrl_states.root_quat = Eigen::Quaterniond(
                                   msg->data[43], // w
                                   msg->data[40], // x
                                   msg->data[41], // y
                                   msg->data[42]  // z
                                   )
                                   .normalized(); // 添加归一化
    // ROS_INFO("root_quat = %f, %f, %f, %f", a1_ctrl_states.root_quat.w(), a1_ctrl_states.root_quat.x(), a1_ctrl_states.root_quat.y(), a1_ctrl_states.root_quat.z());
    // // ======================= 运动学计算 =======================
    a1_ctrl_states.root_rot_mat = a1_ctrl_states.root_quat.toRotationMatrix();
    a1_ctrl_states.root_euler = Utils::quat_to_euler(a1_ctrl_states.root_quat);
    const double yaw_angle = a1_ctrl_states.root_euler[2];
    a1_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());

    // // 计算足端运动学（与原代码逻辑一致）
    // 遍历四条腿进行运动学计算
    for (int leg_id = 0; leg_id < NUM_LEG; ++leg_id)
    {
        // 计算每条腿的关节偏移量(每条腿3个关节)
        const int joint_offset = 3 * leg_id;

        // 使用正向运动学计算足端位置(相对于机身坐标系)
        a1_ctrl_states.foot_pos_rel.block<3, 1>(0, leg_id) = a1_kin.fk(
            a1_ctrl_states.joint_pos.segment<3>(joint_offset), // 当前腿的关节角度
            rho_opt_list[leg_id], rho_fix_list[leg_id]);       // 腿部优化参数和固定参数
        // ROS_INFO("foot_pos_rel: %f, %f, %f", a1_ctrl_states.foot_pos_rel(0, leg_id), a1_ctrl_states.foot_pos_rel(1, leg_id), a1_ctrl_states.foot_pos_rel(2, leg_id));
        // 计算雅可比矩阵(用于速度映射)
        a1_ctrl_states.j_foot.block<3, 3>(3 * leg_id, 3 * leg_id) = a1_kin.jac(
            a1_ctrl_states.joint_pos.segment<3>(joint_offset),
            rho_opt_list[leg_id], rho_fix_list[leg_id]);
        // 如果需要打印矩阵，使用 Eigen 的输出方式
        // ROS_INFO_STREAM("j_foot:\n" << a1_ctrl_states.j_foot);
        // 提取当前腿的雅可比矩阵块和关节速度
        Eigen::Matrix3d jac_block = a1_ctrl_states.j_foot.block<3, 3>(3 * leg_id, 3 * leg_id);
        Eigen::Vector3d joint_vel_block = a1_ctrl_states.joint_vel.segment<3>(joint_offset);
        // 计算足端速度(相对于机身坐标系)
        a1_ctrl_states.foot_vel_rel.block<3, 1>(0, leg_id) = jac_block * joint_vel_block;

        // 将足端位置和速度从机身坐标系转换到机身水平坐标系
        a1_ctrl_states.foot_pos_abs.block<3, 1>(0, leg_id) = a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_pos_rel.block<3, 1>(0, leg_id);
        a1_ctrl_states.foot_vel_abs.block<3, 1>(0, leg_id) = a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_vel_rel.block<3, 1>(0, leg_id);

        // // 将足端位置和速度从机身水平坐标系转换到世界坐标系
        a1_ctrl_states.foot_pos_world.block<3, 1>(0, leg_id) = a1_ctrl_states.foot_pos_abs.block<3, 1>(0, leg_id) + a1_ctrl_states.root_pos;
        a1_ctrl_states.foot_vel_world.block<3, 1>(0, leg_id) = a1_ctrl_states.foot_vel_abs.block<3, 1>(0, leg_id) + a1_ctrl_states.root_lin_vel;
        //     ROS_INFO("foot_pos_world: %f, %f, %f", a1_ctrl_states.foot_pos_world(0, leg_id), a1_ctrl_states.foot_pos_world(1, leg_id), a1_ctrl_states.foot_pos_world(2, leg_id));
        //     ROS_INFO("foot_vel_world: %f, %f, %f", a1_ctrl_states.foot_vel_world(0, leg_id), a1_ctrl_states.foot_vel_world(1, leg_id), a1_ctrl_states.foot_vel_world(2, leg_id));
    }

    // 线加速度（msg.data[56-58]: x, y, z）
    // a1_ctrl_states.imu_acc << msg->data[56], msg->data[57], msg->data[58];
    a1_ctrl_states.imu_acc = Eigen::Vector3d(
        acc_x.CalculateAverage(msg->data[56]),
        acc_y.CalculateAverage(msg->data[57]),
        acc_z.CalculateAverage(msg->data[58]));
    // 角速度（msg.data[47-49]: x, y, z）
    a1_ctrl_states.imu_ang_vel = Eigen::Vector3d(
        gyro_x.CalculateAverage(msg->data[47]),
        gyro_y.CalculateAverage(msg->data[48]),
        gyro_z.CalculateAverage(msg->data[49])
         );
    a1_ctrl_states.root_ang_vel = a1_ctrl_states.root_rot_mat * a1_ctrl_states.imu_ang_vel;

    // ======================= 关节数据解析 =======================
    // 消息中关节顺序定义（每条腿按 calf, hip, thigh 连续存放）:
    // [0]FL_calf, [1]FL_hip, [2]FL_thigh,
    // [3]FR_calf, [4]FR_hip, [5]FR_thigh,
    // [6]RL_calf, [7]RL_hip, [8]RL_thigh,
    // [9]RR_calf, [10]RR_hip, [11]RR_thigh

    // 但代码中关节索引顺序为（每条腿按 hip, thigh, calf 存放）:
    // a1_ctrl_states.joint_pos[0] = FL_hip
    // a1_ctrl_states.joint_pos[1] = FL_thigh
    // a1_ctrl_states.joint_pos[2] = FL_calf
    // a1_ctrl_states.joint_pos[3] = FR_hip
    // ... 其他腿同理

    // 因此需要将消息索引转换为代码索引
    const std::vector<size_t> msg_to_code_idx = {
        // FL: msg[0(calf),1(hip),2(thigh)] → code[2(calf),0(hip),1(thigh)]
        2, 0, 1, // FL_hip=msg[1], FL_thigh=msg[2], FL_calf=msg[0]

        // FR: msg[3(calf),4(hip),5(thigh)] → code[5(hip),6(thigh),4(calf)]
        5, 3, 4, // FR_hip=msg[4], FR_thigh=msg[5], FR_calf=msg[3]

        // RL: msg[6(calf),7(hip),8(thigh)] → code[8(hip),9(thigh),7(calf)]
        8, 6, 7, // RL_hip=msg[7], RL_thigh=msg[8], RL_calf=msg[6]

        // RR: msg[9(calf),10(hip),11(thigh)] → code[11(hip),12(thigh),10(calf)]
        11, 9, 10 // RR_hip=msg[10], RR_thigh=msg[11], RR_calf=msg[9]
    };

    // 解析关节位置（msg.data[0-11]）
    for (int i = 0; i < 12; ++i)
    {
        const size_t code_idx = msg_to_code_idx[i];
        a1_ctrl_states.joint_pos[code_idx] = msg->data[i];
        // ROS_INFO("joint_pos[%d] = %f", code_idx, a1_ctrl_states.joint_pos[code_idx]);
    }

    // 解析关节速度（msg.data[12-23]）
    for (int i = 0; i < 12; ++i)
    {
        const size_t code_idx = msg_to_code_idx[i];
        a1_ctrl_states.joint_vel[code_idx] = msg->data[12 + i];
        // ROS_INFO("joint_vel[%d] = %f", code_idx, a1_ctrl_states.joint_vel[code_idx]);
    }

    // ======================= 足端力（4个） =======================
    // msg.data[36-39]: FL, FR, RL, RR
    for (int i = 0; i < 4; ++i)
    {
        a1_ctrl_states.foot_force[i] = msg->data[36 + i];
        ROS_INFO("foot_force[%d] = %f", i, a1_ctrl_states.foot_force[i]);
    }
    /*
    机身位置：a1_ctrl_states.root_pos和机身速度a1_ctrl_states.root_lin_vel均是EKF估计得到。
    只需获得root_ang_vel机身转动角速度和imu_acc线加速度。
    */
    // 位置（msg.data[44-46]: x, y, z）
    // a1_ctrl_states.root_pos << msg->data[44], msg->data[45], msg->data[46];
    // // ======================= 速度信息 =======================
    // 线速度（msg.data[50-52]: x, y, z）
    // a1_ctrl_states.root_lin_vel << msg->data[50], msg->data[51], msg->data[52];
    // 
    // // ======================= 加速度信息 =======================
    // // 角加速度（msg.data[53-55]: x, y, z）
}
int main(int argc, char *argv[])
{
    
    ros::init(argc,argv,"sub_node");
    ros::NodeHandle nh;
    GazeboA1ROS gazebo_a1_ros(nh);
    ros::Subscriber sub = nh.subscribe("simulation_data", 10, &GazeboA1ROS::sub_callback, &gazebo_a1_ros);
    while(ros::ok())
    {
        ros::spinOnce();
    } 
    return 0;
}
