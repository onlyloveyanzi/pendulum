#ifndef CONTROLLER_MPC_H  
#define CONTROLLER_MPC_H  

#include "controller_base.h"  
#include "mpc_solver/mpc_solver.h"  

const int _mpc_window = 20;  

class MPCInvertedPendulumController : public InvertedPendulumController  
{  
public:  
    MPCInvertedPendulumController(std::shared_ptr<rclcpp::Node> node,  
                                  const Eigen::Matrix4d &A,  
                                  const Eigen::Matrix<double, 4, 1> &B,  
                                  const Eigen::Matrix4d &Q,  
                                  const Eigen::Matrix<double, 1, 1> &R,  
                                  double dt)  
        : InvertedPendulumController(node), A_(A), B_(B), Q_(Q), R_(R), dt_(dt)  
    {  
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MPCInvertedPendulumController"), "A:\n" << A_);  
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MPCInvertedPendulumController"), "B:\n" << B_);  
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MPCInvertedPendulumController"), "Q:\n" << Q_);  
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MPCInvertedPendulumController"), "R:\n" << R_);  
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MPCInvertedPendulumController"), "dt:\n" << dt_);  

        mpc_solver = std::make_shared<MpcSolver<4, 1, _mpc_window>>(A_, B_, Q_, R_, dt);  
        mpc_solver->updateReference(desired_state);  
    }  

    double get_output() override;  

private:  
    Eigen::Matrix4d A_;  
    Eigen::Matrix<double, 4, 1> B_;  
    Eigen::Matrix4d Q_;  
    Eigen::Matrix<double, 1, 1> R_;  
    Eigen::Matrix<double, 1, 1> u;  
    double dt_;  
    std::shared_ptr<MpcSolver<4, 1, _mpc_window>> mpc_solver;  
};  

#endif