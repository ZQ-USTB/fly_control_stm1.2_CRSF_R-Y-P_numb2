# 🚁 飞控系统核心固件 (Flight Control System Firmware)
> **High-Real-Time Flight Control System based on STM32H743**

本项目基于 STM32H7 系列高性能微控制器开发，集成了高实时性多任务调度架构与工业级信号处理算法，专为多自由度飞行器提供高带宽、低延迟的姿态控制与航向锁定解决方案。

---

## 🛡️ 核心算法：二阶巴特沃斯滤波 (Butterworth Filter)
针对传感器高频噪声与电机震动干扰，系统在 `algorithm/biquad_filter.c` 中设计了基于 Biquad（双二阶）拓扑结构的预处理模块：
* **系统架构**：标准二阶巴特沃斯低通滤波器。
* **参数标定**：品质因数 (Q) 严格锁定为 0.7071，确保通带内幅频响应的最平坦特性 (Maximally Flat)。
* **动态自适应**：支持运行期实时重配采样率与截止频率，在信号平滑度与相位延迟 (Phase Delay) 之间实现最优折中。
* **工程应用**：显著衰减 BMI088 宽带噪声，为后续姿态融合算法提供高信噪比 (SNR) 数据源。

---

## 🎮 控制核心：3-DOF 串级 PID 架构
系统实现了涵盖俯仰 (Pitch)、横滚 (Roll) 与偏航 (Yaw) 的三自由度闭环控制律。

### 🔄 串级控制逻辑 (Cascade PID)
采用“角度外环 + 角速度内环”的经典双环嵌套架构：
1. **外环 (Angle Loop)**：响应目标姿态指令，计算并输出期望角速度。
2. **内环 (Rate Loop)**：高频跟踪角速度指令，抑制平台扰动，输出执行器控制量。

### 🧭 增强型航向 (Yaw) 锁定逻辑
针对偏航角在极点处的数值突变问题，在 `algorithm/pid.c` 中独立封装了 `yaw_PID_calc` 函数：
* **过零追踪算法**：内置越界自适应逻辑。当航向角在 ±180° 边界发生跳转时，算法可智能解算最短旋转路径，消除边界震荡。
* **锁头性能**：结合高精度陀螺仪积分补偿，在强干扰下依然保持刚性航向锁定。

---

## 🧩 实时系统：FreeRTOS 任务调度
系统采用多优先级抢占式 RTOS 内核，确保高频控制环路的确定性延迟 (Deterministic Latency)。

| 任务名称 | 优先级 | 调度频率 | 职责描述 |
| :--- | :--- | :--- | :--- |
| **INS_task** | `Real-Time` | 1000Hz | IMU 数据突发读取、EKF/Mahony 姿态解算更新 |
| **Fly_ctrl_task** | `High` | 500Hz | 遥控协议解析、串级 PID 解算、作动器指令映射 |
| **Default_task** | `Normal` | 10Hz | 系统状态监控、看门狗喂狗与后台心跳 |

---

## 🛠️ 硬件架构与构建系统
* **计算平台**：STM32H743XX (480MHz Cortex-M7)，开启硬件双精度 FPU，保障复杂矩阵与浮点运算的执行效率。
* **传感与执行**：原生适配 BMI088 工业级 IMU；利用硬件 Timer 阵列输出高分辨率 PWM 信号驱动执行机构。
* **构建工具链**：**抛弃传统 IDE 绑定，全面采用 VS Code + CMake + ARM GCC 的现代化跨平台编译链**。代码结构清晰，严格遵循 0 Error, 0 Warning 的代码规范。

---

## 📈 调试与校准
* **动态参数注入**：支持通过物理遥控通道实时切换飞行模式，或进行 PID 增益的步进调优。
* **中值补偿机制 (Bias Calibration)**：内置软校准逻辑，支持在运行态手动微调传感器与执行器的零位偏差。

---
<p align="right">
  <i>System Architecture & Development by Chentao Zhu</i><br>
  <i>Firmware Version: v1.1.3 | Toolchain: CMake/GCC</i>
</p>
