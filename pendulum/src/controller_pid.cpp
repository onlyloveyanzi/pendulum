#include <controller_pid.h>

double PIDInvertedPendulumController::get_output()
{
    // 计算误差
    double F_ff = 0;
    // double F_ff = computeFeedforward(current_state(2), current_state(3));
    // double F_trans = 0;
    double F_trans = transController->compute(current_state(0) - desired_state(0), dt_);
    double F_angle = angleController->compute(desired_state(2) - current_state(2), dt_);
    double total = F_angle + F_trans + F_ff;
    return total;
}

// 前馈控制量计算函数

double PIDInvertedPendulumController::computeFeedforward(double theta, double theta_dot)
{
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);
    // 防止除以零 (角度接近±90°时保护)
    if (fabs(cos_theta) < 1e-3)
        cos_theta = 1e-3 * (cos_theta > 0 ? 1 : -1);
    // 前馈力计算公式
    return ((model.M + model.m) * model.g * sin_theta - model.m * model.l * pow(theta_dot, 2) * sin_theta) / cos_theta;
}

// double computeFeedforward(
//     double theta, double theta_dot, double theta_ddot,
//     double x_ref, double x, double dx_ref, double dx,
//     double Kp_x, double Kd_x, double M, double m, double l, double g)
// {
//     // 计算目标加速度
//     double x_err = x_ref - x;
//     double dx_err = dx_ref - dx;
//     double ddx_ref = Kp_x * x_err + Kd_x * dx_err;

//     // 平衡补偿项
//     double balance_ff = ((M + m) * g * sin(theta)) / cos(theta);

//     // 平移补偿项
//     double translation_ff = (M + m) * ddx_ref - m * l * theta_ddot * cos(theta) + m * l * pow(theta_dot, 2) * sin(theta);

//     return balance_ff + translation_ff;
// }