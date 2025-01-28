以下是二级倒立摆的详细动力学建模步骤（拉格朗日法），以及如何将回答保存为图片或其他格式的方法：

---

### **二级倒立摆动力学建模（详细步骤）**

#### **1. 系统描述**
- **小车质量**：\( M \)
- **第一摆杆**：质量 \( m_1 \)，长度 \( 2l_1 \)，质心到铰接点距离 \( l_1 \)
- **第二摆杆**：质量 \( m_2 \)，长度 \( 2l_2 \)，质心到铰接点距离 \( l_2 \)
- **外力**：\( F \) 作用在小车上（水平方向）
- **广义坐标**：小车位置 \( x \)，第一摆杆角度 \( \theta_1 \)，第二摆杆角度 \( \theta_2 \)
- **重力加速度**：\( g \)

---

#### **2. 动能与势能计算**
##### **小车动能**：
\[ T_{\text{cart}} = \frac{1}{2} M \dot{x}^2 \]

##### **第一摆杆动能**：
- **质心坐标**：
  \[
  x_1 = x + l_1 \sin\theta_1, \quad y_1 = l_1 \cos\theta_1
  \]
- **质心速度**：
  \[
  \dot{x}_1 = \dot{x} + l_1 \dot{\theta}_1 \cos\theta_1, \quad \dot{y}_1 = -l_1 \dot{\theta}_1 \sin\theta_1
  \]
- **平动动能**：
  \[
  T_{\text{pend1, trans}} = \frac{1}{2} m_1 \left( \dot{x}_1^2 + \dot{y}_1^2 \right) = \frac{1}{2} m_1 \left[ (\dot{x} + l_1 \dot{\theta}_1 \cos\theta_1)^2 + (l_1 \dot{\theta}_1 \sin\theta_1)^2 \right]
  \]
- **转动动能**：
  \[
  T_{\text{pend1, rot}} = \frac{1}{2} I_1 \dot{\theta}_1^2, \quad I_1 = \frac{1}{3} m_1 (2l_1)^2 = \frac{4}{3} m_1 l_1^2
  \]
- **总动能**:
  \[
  T_{\text{pend1}} = \frac{1}{2} m_1 \left( \dot{x}^2 + 2 l_1 \dot{x} \dot{\theta}_1 \cos\theta_1 + l_1^2 \dot{\theta}_1^2 \right) + \frac{1}{2} I_1 \dot{\theta}_1^2
  \]

##### **第二摆杆动能**：
- **质心坐标**（相对于第一摆杆末端）：
  \[
  x_2 = x + 2l_1 \sin\theta_1 + l_2 \sin\theta_2, \quad y_2 = 2l_1 \cos\theta_1 + l_2 \cos\theta_2
  \]
- **质心速度**：
  \[
  \dot{x}_2 = \dot{x} + 2l_1 \dot{\theta}_1 \cos\theta_1 + l_2 \dot{\theta}_2 \cos\theta_2
  \]
  \[
  \dot{y}_2 = -2l_1 \dot{\theta}_1 \sin\theta_1 - l_2 \dot{\theta}_2 \sin\theta_2
  \]
- **平动动能**：
  \[
  T_{\text{pend2, trans}} = \frac{1}{2} m_2 \left( \dot{x}_2^2 + \dot{y}_2^2 \right)
  \]
  \[
  = \frac{1}{2} m_2 \left[ (\dot{x} + 2l_1 \dot{\theta}_1 \cos\theta_1 + l_2 \dot{\theta}_2 \cos\theta_2)^2 + (-2l_1 \dot{\theta}_1 \sin\theta_1 - l_2 \dot{\theta}_2 \sin\theta_2)^2 \right]
  \]
- **转动动能**：
  \[
  T_{\text{pend2, rot}} = \frac{1}{2} I_2 \dot{\theta}_2^2, \quad I_2 = \frac{1}{3} m_2 (2l_2)^2 = \frac{4}{3} m_2 l_2^2
  \]
- **总动能**：
  \[
  T_{\text{pend2}} = \frac{1}{2} m_2 \left( \dot{x}^2 + 4 l_1 \dot{x} \dot{\theta}_1 \cos\theta_1 + 4 l_1^2 \dot{\theta}_1^2 + 2 l_2 \dot{x} \dot{\theta}_2 \cos\theta_2 + 4 l_1 l_2 \dot{\theta}_1 \dot{\theta}_2 \cos(\theta_1 - \theta_2) + l_2^2 \dot{\theta}_2^2 \right) + \frac{1}{2} I_2 \dot{\theta}_2^2
  \]

##### **系统总动能**：
\[
T = T_{\text{cart}} + T_{\text{pend1}} + T_{\text{pend2}}
\]

##### **系统势能**：
\[
V = m_1 g y_1 + m_2 g y_2 = m_1 g l_1 \cos\theta_1 + m_2 g \left( 2l_1 \cos\theta_1 + l_2 \cos\theta_2 \right)
\]

---

#### **3. 拉格朗日函数**
\[
L = T - V
\]

---

#### **4. 拉格朗日方程**
对每个广义坐标 \( q_i \in \{ x, \theta_1, \theta_2 \} \)，应用方程：
\[
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}_i} \right) - \frac{\partial L}{\partial q_i} = Q_i
\]
其中广义力：
- \( Q_x = F \)（水平外力）
- \( Q_{\theta_1} = 0 \)，\( Q_{\theta_2} = 0 \)

---

#### **5. 对广义坐标 \( x \) 的方程**
\[
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{x}} \right) - \frac{\partial L}{\partial x} = F
\]
展开后：
\[
(M + m_1 + m_2) \ddot{x} + m_1 l_1 \ddot{\theta}_1 \cos\theta_1 + m_2 l_2 \ddot{\theta}_2 \cos\theta_2 - m_1 l_1 \dot{\theta}_1^2 \sin\theta_1 - m_2 l_2 \dot{\theta}_2^2 \sin\theta_2 = F
\]

---

#### **6. 对广义坐标 \( \theta_1 \) 的方程**
\[
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}_1} \right) - \frac{\partial L}{\partial \theta_1} = 0
\]
展开后：
\[
m_1 l_1 \ddot{x} \cos\theta_1 + (I_1 + m_1 l_1^2) \ddot{\theta}_1 + 2 m_2 l_1 \ddot{x} \cos\theta_1 + 4 m_2 l_1^2 \ddot{\theta}_1 - m_1 g l_1 \sin\theta_1 - 2 m_2 g l_1 \sin\theta_1 = 0
\]

---

#### **7. 对广义坐标 \( \theta_2 \) 的方程**
\[
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}_2} \right) - \frac{\partial L}{\partial \theta_2} = 0
\]
展开后：
\[
m_2 l_2 \ddot{x} \cos\theta_2 + (I_2 + m_2 l_2^2) \ddot{\theta}_2 + 2 m_2 l_1 l_2 \ddot{\theta}_1 \cos(\theta_1 - \theta_2) - m_2 g l_2 \sin\theta_2 = 0
\]

---

#### **8. 线性化**
在平衡点 \( \theta_1 \approx 0 \)、\( \theta_2 \approx 0 \) 附近，假设：
- \( \sin\theta_i \approx \theta_i \)
- \( \cos\theta_i \approx 1 \)
- \( \dot{\theta}_i^2 \approx 0 \)
- \( \cos(\theta_1 - \theta_2) \approx 1 \)

线性化后的方程：
\[
\begin{cases}
(M + m_1 + m_2) \ddot{x} + m_1 l_1 \ddot{\theta}_1 + m_2 l_2 \ddot{\theta}_2 = F \\
(m_1 l_1 + 2 m_2 l_1) \ddot{x} + (I_1 + m_1 l_1^2 + 4 m_2 l_1^2) \ddot{\theta}_1 - (m_1 g l_1 + 2 m_2 g l_1) \theta_1 = 0 \\
m_2 l_2 \ddot{x} + (I_2 + m_2 l_2^2) \ddot{\theta}_2 - m_2 g l_2 \theta_2 = 0
\end{cases}
\]

---

### **将回答保存为图片或其他格式**

#### **方法 1：截图保存**
1. **全屏截图**：
   - Windows：按 `Win + Shift + S`，选择区域后保存。
   - Mac：按 `Cmd + Shift + 4`，拖动选择区域。
2. **浏览器打印为PDF**：
   - 按 `Ctrl + P`（Windows）或 `Cmd + P`（Mac）。
   - 选择“另存为PDF”。

#### **方法 2：复制到文档**
1. 复制文本到Word或Markdown编辑器。
2. 调整格式后导出为PDF或图片。

#### **方法 3：使用在线工具**
1. 访问 [Carbon.now.sh](https://carbon.now.sh)，粘贴代码生成美观的代码图片。
2. 使用 [Text to Image Converter](https://text-image.com/) 将文本转为图片。

---

**最终效果示例（保存为图片）**：  
![二级倒立摆动力学方程](https://via.placeholder.com/800x600.png?text=Dynamic+Equations+of+Double+Inverted+Pendulum)

--- 

如需进一步简化或符号计算代码（如MATLAB/Python），请告知！