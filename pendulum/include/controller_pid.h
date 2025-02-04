#pragma once

#include <controller_base.h>
#include <pid.h>

class PIDInvertedPendulumController : public InvertedPendulumController
{

public:
    PIDInvertedPendulumController(std::shared_ptr<rclcpp::Node> node,
                                  const Eigen::Matrix4d &A,
                                  const Eigen::Matrix<double, 4, 1> &B,
                                  double dt) : InvertedPendulumController(node), A_(A), B_(B), dt_(dt)
    {
        // RCLCPP_INFO(rclcpp::get_logger("PIDInvertedPendulumController"));
        // RCLCPP_INFO(rclcpp::get_logger("PIDInvertedPendulumController"), "Kp: %f", Kp);
        angleController = new PIDController(80.0, 5, 15.0);
        transController = new PIDController(2, 0.1, 5.0);
    }

    double get_output() override;
    double computeFeedforward(double, double);
    double computeFeedforwardNonlinear();

private:
    Eigen::Matrix4d A_;
    Eigen::Matrix<double, 4, 1> B_;
    double dt_;
    pendulumModel model;

    PIDController *transController;
    PIDController *angleController;
};