#pragma once

#include <controller_base.h>

class PIDInvertedPendulumController : public InvertedPendulumController
{

public:
    PIDInvertedPendulumController(std::shared_ptr<rclcpp::Node> node,
                                  const Eigen::Matrix4d &A,
                                  const Eigen::Matrix<double, 4, 1> &B,
                                  double dt, double Kp, double Ki, double Kd) : InvertedPendulumController(node), A_(A), B_(B), dt_(dt), Kp_(Kp), Ki_(Ki), Kd_(Kd)
    {
        // RCLCPP_INFO(rclcpp::get_logger("PIDInvertedPendulumController"));
        RCLCPP_INFO(rclcpp::get_logger("PIDInvertedPendulumController"), "Kp: %f", Kp);
    }

    double get_output() override;
    void update();
    double compute(double setpoint, double measurement);

private:
    Eigen::Matrix4d A_;
    Eigen::Matrix<double, 4, 1> B_;
    double dt_;
    double Kp_, Ki_, Kd_;
    double max_out = 999999;
    double integral = 0;
    double prev_err = 0;
};