#!/usr/bin/env python3  
import numpy as np  
import rclpy  
from rclpy.node import Node  
from rclpy.parameter import Parameter  

from lqr import LQRInvertedPendulumController  
from mpc import MPCInvertedPendulumController  

class InvertedPendulumControllerNode(Node):  
    def __init__(self):  
        super().__init__('inverted_pendulum_controller')  

        # Get the controller type from parameters  
        self.declare_parameter('controller', 'LQR')  
        controller_type = self.get_parameter('controller').get_parameter_value().string_value  

        # Cartpole parameters  
        M = 2.0  # kg  
        m = 0.1  # kg  
        g = 9.8  # m/s²  
        l = 0.5  # meters  
        l /= 2  
        I = 1 / 3 * m * l * l  
        b = 0.0  # N·m·s  

        P = (M + m) * I + M * m * l * l  

        # System matrices  
        A = np.array([[0, 1, 0, 0],  
                      [0, -b * (I + m * l * l) / P, m * m * g * l * l / P, 0],  
                      [0, 0, 0, 1],  
                      [0, -b * m * l / P, m * g * l * (M + m) / P, 0]])  

        B = np.array([[0],  
                      [(I + m * l * l) / P],  
                      [0],  
                      [m * l / P]])  

        Q = np.array([[10.0, 0, 0, 0],  
                      [0, 10.0, 0, 0],  
                      [0, 0, 10.0, 0],  
                      [0, 0, 0, 10.0]])  

        R = np.array([[0.1]])  

        x_ref = np.array([0.0, 0.0, 0.0, 0.0])  

        if controller_type.lower() == "lqr":  
            self.controller = LQRInvertedPendulumController(A, B, Q, R, x_ref=x_ref)  
        elif controller_type.lower() == "mpc":  
            dt = 0.02  
            N = 200  
            self.controller = MPCInvertedPendulumController(A, B, Q, R, dt, N, x_ref=x_ref)  
        else:  
            raise ValueError("Invalid controller type specified.")  

        self.run()  

    def run(self):  
        # Implement the run logic for the controller here  
        # This could involve a loop or a timer to periodically update the controller  
        self.get_logger().info("Running the controller...")  
        # Example: self.controller.run() if it has a run method  

def main(args=None):  
    rclpy.init(args=args)  
    controller_node = InvertedPendulumControllerNode()  

    try:  
        rclpy.spin(controller_node)  
    except KeyboardInterrupt:  
        pass  
    finally:  
        controller_node.destroy_node()  
        rclpy.shutdown()  

if __name__ == '__main__':  
    main()