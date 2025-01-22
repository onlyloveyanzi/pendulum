#ifndef CONTROLLER_LQR_H  
#define CONTROLLER_LQR_H  

#include "controller_base.h"  

class LQRInvertedPendulumController : public InvertedPendulumController  
{  
public:  
    LQRInvertedPendulumController(std::shared_ptr<rclcpp::Node> node,  
                                  const Eigen::Matrix4d &A,  
                                  const Eigen::Matrix<double, 4, 1> &B,  
                                  const Eigen::Matrix4d &Q,  
                                  const Eigen::Matrix<double, 1, 1> &R)  
        : InvertedPendulumController(node), A_(A), B_(B), Q_(Q), R_(R)  
    {  
        K = computeLQR();  
        RCLCPP_INFO_STREAM(rclcpp::get_logger("LQRInvertedPendulumController"), "LQR Gain Matrix K:\n" << K);  
    }  

    double get_output() override;  

private:  
    Eigen::Matrix<double, 4, 1> K;  
    Eigen::Matrix4d A_;  
    Eigen::Matrix<double, 4, 1> B_;  
    Eigen::Matrix4d Q_;  
    Eigen::Matrix<double, 1, 1> R_;  

    Eigen::Matrix<double, 4, 1> computeLQR();  
};  

#endif