# 电池SOC估计-卡尔曼滤波

[English](./README.md)

这个小项目来自于我大学毕业设计的放着模拟部分，目的是估计锂电池的荷电状态(SOC)。主要完成了扩展卡尔曼滤波(EKF)的实验、参数辨识和仿真。完成无迹卡尔曼滤波(UKF)仿真需要感谢我的朋友顾鹏程的贡献。BBDST的工作环境也得益于我的师兄蒋聪的帮助。谢谢我的小伙伴们。♥

## General Content

项目中包括锂电池模型建立、参数辨识与验证、SOC估计采用扩展卡尔曼滤波(EKF)，使用了两种方式实现：

1. Simulinks(EKF only)
2. 脚本(包含EKF和UKF)

## 第一次尝试

- 该模型的输入包括电流和电压来自于HPPC（混合脉冲功率特性）测试的电池数据。
- 文件EKFSim_R2016中使用了Thevenin等效电路模型和扩展卡尔曼滤波器，结构如下。

![Simulink](./imgs/simulink.png)
<p align="center">EKFSim_R2016文件的结构</p>

- 估计曲线在电流脉冲区域有明显的发散，在恒流放电区域逐渐收敛到真实值。
- SOC和Up(RC元件在Thevenin ECM中的电压)的估计值同步变化，这是由于它们处于相同的状态向量中而，这可以在功能块“EKF”中看到。

![States Output](./imgs/Output.png)
<p align="center">EKFSim_R2016文件的输出</p>

- 卡尔曼滤波根据UL(负载电压)的观测值与预测值的差，对包括SOC和Up在内的状态组进行更新。代码表达如下。  

```matlab
X_upd = X_pre + K*(UL_obs-UL_pre);
```

![UL curves](./imgs/UL.png)
<p align="center">UL的变化</p>

## 改进

- 在改进之后，模块之间的I/O关系变得更加清晰，相应的Simulink文件命名为Improved_EKFSim。

![Improvement](./imgs/ImprovedSim.jpg)
<p align="center">改进后的结构</p>

- 脚本文件可以仿真在BBDST(北京公交车动态街道测试)工况和带有观测噪声的恒流工况下的锂离子电池放电过程，利用EKF/UKF方法估算电池荷电状态。

```matlab
function main(Work_modes, SOC_est_init)
```

- 主函数需要两个参数，Work_mode:工况1- BBDST模式，2-恒流模式，SOC_est_init: SOC估计值的初始值，默认为1。如果只给出一个参数，它将被赋给Work_mode。在命令窗口中输入`main()`或`main(1)`或`main(1,1)`，结果如下。

![States estimation curve](./imgs/SimResult.jpg)
<p align="center">结果输出曲线</p>
