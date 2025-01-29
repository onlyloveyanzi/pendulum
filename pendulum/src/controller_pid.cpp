#include <controller_pid.h>
#include <iostream>

double PIDInvertedPendulumController::get_output()
{
    // 计算误差
    double F_ff = 0;
    // double F_ff = computeFeedforward(current_state(2), current_state(3));
    // double F_ff = computeFeedforwardNonlinear();
    // double F_trans = 0;
    double F_trans = transController->compute(current_state(0) - desired_state(0), dt_);
    // double F_angle = 0;
    double F_angle = angleController->compute(current_state(2) - desired_state(2), dt_);
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

double PIDInvertedPendulumController::computeFeedforwardNonlinear()
{
    // 1. 计算目标加速度
    double x_error = current_state[0] - desired_state[0];
    double dx_error = current_state[1] - desired_state[1];
    double ddx_ref = 3.0 * x_error + 2.0 * dx_error;

    // 2. 非线性平衡补偿项
    double sin_theta = sin(current_state[2]);
    double cos_theta = cos(current_state[2]);
    RCLCPP_INFO(rclcpp::get_logger("InvertedPendulumController"), "sin theta: %f m", sin_theta);
    RCLCPP_INFO(rclcpp::get_logger("InvertedPendulumController"), "cos theta: %f m", cos_theta);
    if (fabs(cos_theta) < 1e-3)
        cos_theta = 1e-3 * (cos_theta > 0 ? 1 : -1); // 防止除以零
    double balance_ff = ((model.M + model.m) * model.g * sin_theta - model.m * model.l * pow(current_state[3], 2) * sin_theta) / cos_theta;

    // 3. 平移补偿项
    double translation_ff = (model.M + model.m) * ddx_ref;

    return balance_ff + translation_ff;
}