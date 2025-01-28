#include <controller_pid.h>


double PIDInvertedPendulumController::get_output()
{
    // 计算误差
    double error = desired_state(0) - current_state(0);
    return error;
}

double PIDInvertedPendulumController::compute(double setpoint, double measurement)
{
    double err = setpoint - measurement;
    integral += err * dt_;
    double derivative = (err - prev_err) / dt_;
    prev_err = err;

    double output = Kp_ * err + Ki_ * integral + Kd_ * derivative;

    // 抗饱和处理
    if (output > max_out)
        return max_out;
    if (output < -max_out)
        return -max_out;
    return output;
}

void PIDInvertedPendulumController::update(double F, double dt)
{
    // 简化的线性化动力学方程
    const double sin_theta = sin(current_state[2]);
    const double cos_theta = cos(current_state[2]);

    // current_state = A * current_state + B * F;  

    // 欧拉积分
    // state.dx += x_accel * dt;
    // state.x += state.dx * dt;
    // state.dtheta += theta_accel * dt;
    // state.theta += state.dtheta * dt;
}