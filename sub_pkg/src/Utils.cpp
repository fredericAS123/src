//
// Created by shuoy on 10/19/21.
//

#include "Utils.h"

Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}

Eigen::Matrix3d Utils::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
}

// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
Eigen::Matrix3d Utils::pseudo_inverse(const Eigen::Matrix3d &mat) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    // For a non-square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}

//这段代码定义了一个函数 cal_dihedral_angle，用于计算由两个平面所形成的二面角（dihedral angle）。二面角是两个平面之间的夹角，它可以通过两个平面的法向量来计算。函数的输入是两个平面的系数向量 surf_coef_1 和 surf_coef_2，分别表示两个平面的法向量'''
double Utils::cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2) {
    // surface 1: a1 * x + b1 * y + c1 * z + d1 = 0, coef: [a1, b1, c1]
    // surface 2: a2 * x + b2 * y + c2 * z + d2 = 0, coef: [a1, b2, c2]
    double angle_cos =
            abs(surf_coef_1[0] * surf_coef_2[0] + surf_coef_1[1] * surf_coef_2[1] + surf_coef_1[2] * surf_coef_2[2])
            / (sqrt(surf_coef_1[0] * surf_coef_1[0] + surf_coef_1[1] * surf_coef_1[1] + surf_coef_1[2] * surf_coef_1[2]) *
               sqrt(surf_coef_2[0] * surf_coef_2[0] + surf_coef_2[1] * surf_coef_2[1] + surf_coef_2[2] * surf_coef_2[2]));
    return acos(angle_cos);
}

Eigen::Vector3d BezierUtils::get_foot_pos_curve(float t,
                                                 Eigen::Vector3d foot_pos_start,
                                                 Eigen::Vector3d foot_pos_final, 
                                                 double terrain_pitch_angle = 0)
{
    Eigen::Vector3d foot_pos_target;  // 最终计算出的目标足部位置

    // X轴贝塞尔插值
    std::vector<double> bezierX{foot_pos_start(0),
                               foot_pos_start(0),
                               foot_pos_final(0),
                               foot_pos_final(0),
                               foot_pos_final(0)};
    foot_pos_target(0) = bezier_curve(t, bezierX);  // 计算X轴目标位置

    // Y轴贝塞尔插值
    std::vector<double> bezierY{foot_pos_start(1),
                                foot_pos_start(1),
                                foot_pos_final(1),
                                foot_pos_final(1),
                                foot_pos_final(1)};
    foot_pos_target(1) = bezier_curve(t, bezierY);  // 计算Y轴目标位置

    // Z轴贝塞尔插值，带有地面坡度的调整
    std::vector<double> bezierZ{foot_pos_start(2),
                                foot_pos_start(2),
                                foot_pos_final(2),
                                foot_pos_final(2),
                                foot_pos_final(2)};
    bezierZ[1] += FOOT_SWING_CLEARANCE1;  // 在起始位置调整足部清除高度
    bezierZ[2] += FOOT_SWING_CLEARANCE2 + 0.5*sin(terrain_pitch_angle);  // 在目标位置进行调整，加入地面坡度影响
    foot_pos_target(2) = bezier_curve(t, bezierZ);  // 计算Z轴目标位置

    return foot_pos_target;  // 返回计算的目标足部位置
}



double BezierUtils::bezier_curve(double t, const std::vector<double> &P) {
    std::vector<double> coefficients{1, 4, 6, 4, 1};  // 贝塞尔曲线的控制系数
    double y = 0;
    // 遍历所有控制点，计算加权和
    for (int i = 0; i <= bezier_degree; i++) {
        y += coefficients[i] * std::pow(t, i) * std::pow(1 - t, bezier_degree - i) * P[i];
    }
    return y;  // 返回曲线在t位置的值
}
