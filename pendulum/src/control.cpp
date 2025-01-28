#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <memory>

#include "controller_base.h"
#include "controller_lqr.h"
#include "controller_mpc.h"

const int control_freq = 100;

const double M = 2.0;
const double m = 0.1;
const double g = 9.8;
const double l = 0.5 / 2.0;
const double I = 1.0 / 3.0 * m * l * l;
const double b = 0.0;
const double P = (M + m) * I + M * m * l * l;

std::string CONTROLLER;

// A and B matrices
Eigen::Matrix4d A;
Eigen::Matrix<double, 4, 1> B;

Eigen::Matrix4d Q;
Eigen::Matrix<double, 1, 1> R;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("controller");

    std::string default_value = "LQR";
    node->declare_parameter("controller", default_value);
    node->get_parameter("controller", CONTROLLER);
    RCLCPP_INFO(node->get_logger(), "Controller: %s", CONTROLLER.c_str());

    rclcpp::Rate loop_rate(control_freq);

    A << 0, 1, 0, 0,
        0, -b * (I + m * l * l) / P, m * m * g * l * l / P, 0,
        0, 0, 0, 1,
        0, -b * m * l / P, m * g * l * (M + m) / P, 0;
    B << 0, (I + m * l * l) / P, 0, m * l / P;

    Q << 10.0, 0.0, 0.0, 0.0,
        0.0, 10.0, 0.0, 0.0,
        0.0, 0.0, 10.0, 0.0,
        0.0, 0.0, 0.0, 10.0;
    R << 0.1;

    std::unique_ptr<InvertedPendulumController> controller1;
    std::unique_ptr<InvertedPendulumController> controller2;
    if (CONTROLLER == "LQR")
    {
        controller1 = std::make_unique<LQRInvertedPendulumController>(node, A, B, Q, R);
    }
    else if (CONTROLLER == "MPC")
    {
        controller1 = std::make_unique<MPCInvertedPendulumController>(node, A, B, Q, R, 0.02);
    }
    else if (CONTROLLER == "PID")
    {
        controller1 = std::make_unique<MPCInvertedPendulumController>(node, A, B, 0.02, 80, 0, 15);
        controller2 = std::make_unique<MPCInvertedPendulumController>(node, A, B, 0.02, 2, 0.1, 5.0);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Controller %s is not available", CONTROLLER.c_str());
        return 1;
    }

    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if(CONTROLLER == "PID"){
            controller1->update();
        }else{
            controller->balance();
        }
        
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}