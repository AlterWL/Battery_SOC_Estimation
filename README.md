# Battery State of Charge Estimation Using Kalman Filter

---

This small project comes from the simulation part of my college graduation design. I mainly finished the experiments, parameters identification and simulation of extended kalman filter(EKF). The completion unscented kalman filter(UKF) simulation is thanks to the contribution from my friend, Pengcheng Gu. The BBDST working condition block is also benefit from the help so my senior, Cong Jiang.

## General Content

Li-Battery model building, parameters identification and verification, SOC estimation using extended kalman filter in Matlab, Simulink.

## First Try

- The inputs of the model include current and voltage from baattery test(HPPC) data.
- Thevenin equivalent circuit model and extended kalman filter are included in the simulation file "EKFBlocks_R2016.slx", of which the structure is shown in the snapshot below.

![Simulink](./imgs/simulink.png)
<p align="center">Structure of EKFBlocks_R2016.slx</p>

- The estimated curve has distinct divergences in the current pulse areas and it converges to the true value in the constant current discharge areas. 
- The estimated SOC and update Up(voltage of RC element in Thevenin ECM) change synchronously due to the same state vector that they are in, that can be seen in the codes in MATLAB Fction block 'EKF'.

![States Output](./imgs/Output.png)
<p align="center">Output of EKFBlocks_R2016.slx</p>

- Kalman flter update of states including SOC and Up, accroding to the difference betweent observed values and predicted values of UL(voltage on the load). The code format of this expression is as following.  

```matlab
X_upd = X_pre + K*(UL_obs-UL_pre);
```

![UL curves](./imgs/UL.png)
<p align="center">UL Variation</p>

- A runable script named 'Thevenin_EKF_SOC.m' has been uploaded for test.
  
- It simulats the constant current discharge process of lithium-ion battery with observation noise and uses EKF method to estimate SOC of the battery.

## Improtment

- After improvement, the I/O relationship between modules becomes more perspicuous.

![Improvement](./imgs/ImprovedSim.jpg)
<p align="center">Structure of EKF_UKF_Thev.slx</p>

- The curve below is result of main simulation function in "main.m", which runs the Matlab function in "EKF_UKF_Thev.m"

![States estimation curve](./imgs/SimResult.jpg)
<p align="center">Result of codes simulation</p>
