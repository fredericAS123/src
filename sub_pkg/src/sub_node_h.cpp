#include "sub_node.h"

GazeboA1ROS::GazeboA1ROS(ros::NodeHandle &_nh) {
    nh = _nh;
leg_offset_x[0] = 0.1805;
leg_offset_x[1] = 0.1805;
leg_offset_x[2] = -0.1805;
leg_offset_x[3] = -0.1805;
leg_offset_y[0] = 0.047;
leg_offset_y[1] = -0.047;
leg_offset_y[2] = 0.047;
leg_offset_y[3] = -0.047;
motor_offset[0] = 0.0838;
motor_offset[1] = -0.0838;
motor_offset[2] = 0.0838;
motor_offset[3] = -0.0838;
upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.21;
lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = LOWER_LEG_LENGTH;

for (int i = 0; i < NUM_LEG; i++) {
    Eigen::VectorXd rho_fix(5);
    rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
    Eigen::VectorXd rho_opt(3);
    rho_opt << 0.0, 0.0, 0.0;
    rho_fix_list.push_back(rho_fix);
    rho_opt_list.push_back(rho_opt);
}

}