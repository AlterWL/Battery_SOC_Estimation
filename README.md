# EKF on SOC estimation

Simulation of SOC estimation using extended kalman filter in Matlab 

## Description

Li-Battery model building, parameters identification and verification, SOC estimation using extended kalman filter in Matlab, Simulink.

## Content
* The inputs of the model include current and voltage from baattery test(HPPC) data.
* Thevenin equivalent circuit model and extended kalman filter are implemented to estimat SOC of a 50Ah NCM li-ion battery of China Aviation Lithium Battery Co.Ltd.
![Simulink block connection diagram](https://github.com/AlterWL/EKF-on-SOC-Estimation/blob/master/simulink.png)
* The estimated curve has distinct divergences in the current pulse areas and it converges to the true value in the constant current discharge areas. 
* The estimated SOC and update Up(voltage of RC element in Thevenin ECM) change synchronously due to the same state vector that they are in, that can be seen in the codes in MATLAB Fction block 'EKF'.
![States estimation curve](https://github.com/AlterWL/EKF-on-SOC-Estimation/blob/master/sim_curves.png)
* Kalman flter update of states including SOC and Up, accroding to the difference betweent observed values and predicted values of UL(voltage on the load). The code format of this expression is as following.

`X_upd = X_pre + K*(UL_obs-UL_pre);`
![UL curves](https://github.com/AlterWL/EKF-on-SOC-Estimation/blob/master/UL.png)
* A runable script named 'Thevenin_EKF_SOC.m' has been uploaded for test. 
* It simulats the constant current discharge process of lithium-ion battery with observation noise and uses EKF method to estimate SOC of the battery.
