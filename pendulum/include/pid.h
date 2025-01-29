#pragma once

#include <iostream>
#include <cmath>

// PID 控制器类
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd) 
        : Kp(Kp), Ki(Ki), Kd(Kd), integral(0), prev_error(0) {}

    double compute(double error, double dt) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }

    void reset() {
        integral = 0;
        prev_error = 0;
    }

private:
    double Kp, Ki, Kd;
    double integral, prev_error;
};