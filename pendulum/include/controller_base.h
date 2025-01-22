#ifndef CONTROLLER_BASE_H  
#define CONTROLLER_BASE_H  

#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/joint_state.hpp>  
#include <std_msgs/msg/float64.hpp>  
#include <Eigen/Dense>  
#include <memory>  

class InvertedPendulumController  
{  
public:  
    InvertedPendulumController(std::shared_ptr<rclcpp::Node> node)  
    {  
        command_pub = node->create_publisher<std_msgs::msg::Float64>("/pendulum/x_controller/command", 10);  
        joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(  
            "/pendulum/joint_states", 10,  
            std::bind(&InvertedPendulumController::jointStateCallback, this, std::placeholders::_1)  
        );  

        current_state << 0.0, 0.0, 0.0, 0.0;  
        desired_state << 0.0, 0.0, 0.0, 0.0;  
    }  

    virtual ~InvertedPendulumController() {}  

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr data)  
    {  
        current_state(0) = data->position[0];  
        current_state(1) = data->velocity[0];  
        current_state(2) = -data->position[1];  
        current_state(3) = -data->velocity[1];  

        RCLCPP_INFO(rclcpp::get_logger("InvertedPendulumController"), "x_pos: %f m", current_state(0));  
        RCLCPP_INFO(rclcpp::get_logger("InvertedPendulumController"), "x_vel: %f m/s", current_state(1));  
        RCLCPP_INFO(rclcpp::get_logger("InvertedPendulumController"), "theta_pos: %f rad", current_state(2));  
        RCLCPP_INFO(rclcpp::get_logger("InvertedPendulumController"), "theta_vel: %f rad/s", current_state(3));  
    }  

    void balance()  
    {  
        double output = get_output();  
        command_msg.data = output;  
        command_pub->publish(command_msg);  
        last_balance_time = rclcpp::Clock().now();  
        RCLCPP_INFO(rclcpp::get_logger("InvertedPendulumController"), "commanding: %f", command_msg.data);  
    }  

    virtual double get_output() = 0;  

protected:  
    Eigen::Vector4d current_state;  
    Eigen::Vector4d desired_state;  
    rclcpp::Time last_balance_time;  

private:  
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_pub;  
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;  
    std_msgs::msg::Float64 command_msg;  
};  

#endif