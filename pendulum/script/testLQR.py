import numpy as np  

# 系统参数  
M = 1.0  
m = 0.1  
l = 0.5  
b = 0 
g = 9.81  
theta_d = np.deg2rad(5)  # 目标角度转换为弧度  

# 矩阵 A  
A = np.array([  
    [0, 1, 0, 0],  
    [0, -b/M, m*g*np.cos(theta_d)/M, 0],  
    [0, 0, 0, 1],  
    [0, -b*np.cos(theta_d)/(M*l), (M+m)*g*np.cos(theta_d)/(M*l), 0]  
])  

# 矩阵 B  
B = np.array([  
    [0],  
    [1/M],  
    [0],  
    [np.cos(theta_d)/(M*l)]  
])

from scipy.linalg import solve_continuous_are  

# 权重矩阵  
Q = np.diag([10, 1, 100, 1])  # 调节状态优先级  
R = np.array([[1]])           # 调节控制输入优先级  

# 求解 Riccati 方程  
P = solve_continuous_are(A, B, Q, R)  

# 计算反馈增益矩阵 K  
K = np.linalg.inv(R) @ B.T @ P

from scipy.integrate import solve_ivp  
import matplotlib.pyplot as plt  

# 系统动力学方程  
def system_dynamics(t, x):  
    u = -K @ x  # 控制输入  
    dxdt = A @ x + B @ u  
    return dxdt.flatten()  

# 初始条件  
x0 = np.array([0, 0, np.deg2rad(10)-theta_d, 0])  # 初始角度为 0 度  

# 仿真时间  
t_span = (0, 100)  

# 求解微分方程  
sol = solve_ivp(system_dynamics, t_span, x0, t_eval=np.linspace(0, 10, 1000))  

# 绘制结果  
plt.figure(figsize=(10, 6))  
plt.plot(sol.t, np.rad2deg(sol.y[2]), label='Angle (deg)')  
plt.axhline(np.rad2deg(theta_d), color='r', linestyle='--', label='Target Angle')  
plt.xlabel('Time (s)')  
plt.ylabel('Angle (deg)')  
plt.legend()  
plt.grid()  
plt.show()