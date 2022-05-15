# Battery State of Charge Estimation Using Kalman Filter

[简体中文](./README_zh_CN.md)

This small project comes from the simulation part of my college graduation design which aimed to estimate the state of charge(SoC) of lithium battery. I mainly finished the experiments, parameters identification and simulation of extended kalman filter(EKF). The completion unscented kalman filter(UKF) simulation needs to thank the contribution from my friend, Pengcheng Gu. And the BBDST working condition block is also benefit from the help of my senior, Cong Jiang. 😘🔋thanks angain to my warm friends.🔋❤

## General Content

Li-Battery model building, parameters identification and verification, SoC estimation using extended kalman filter(EKF) through two ways:

1. Simulinks(EKF only)
2. Scripts(EKF&UKF)

## First Try

- The inputs of the model include current and voltage comes from battery data in HPPC(Hybrid PulsePower Characteristic) test.
- Thevenin equivalent circuit model and extended kalman filter are included in the simulation file "EKFSim_R2016.slx", of which the structure is shown in the snapshot below.

![Simulink](./imgs/simulink.png)
<p align="center">Structure of EKFSim_R2016.slx</p>

- The estimated curve has distinct divergences in the current pulse areas and it converges to the true value in the constant current discharge areas.
- The estimated SoC and update Up(voltage of RC element in Thevenin ECM) change synchronously due to the same state vector that they are in, that can be seen in the function block 'EKF'.

![States Output](./imgs/Output.png)
<p align="center">Output of EKFSim_R2016.slx</p>

- Kalman filter update of states including SoC and Up, according to the difference between observed values and predicted values of UL(voltage on the load). The code format of this expression is as following.  

```matlab
X_upd = X_pre + K*(UL_obs-UL_pre);
```

![UL curves](./imgs/UL.png)
<p align="center">UL Variation</p>

## Improvement

- After improvement, the I/O relationship between modules becomes more perspicuous, the corresponding Simulink file is named Improved_EKFSim.slx.

![Improvement](./imgs/ImprovedSim.jpg)
<p align="center">Improved Structure</p>

- MATLAB scripts simulate discharge process of lithium-ion battery under the BBDST(Beijing Bus Dynamic Street Test) working condition and constant current working condition with observation noise, and uses EKF/UKF method to estimate SoC of the battery.

```matlab
function main(Work_modes, SoC_est_init)
```

- The main function requires two arguments, Work_mode: Mode of working condition 1-BBDST, 2-constant current, SoC_est_init: The initial value of estimated SoC, it's set to 1 by default. If you give just on argument, it will be given to Work_mode.  
type in command window like `main()`or`main(1)`or`main(1,1)`, the result curves will appear as follows.

![States estimation curve](./imgs/SimResult.jpg)
<p align="center">Result Curves</p>

## Related Knowledge

### 1. Thevenin equivalent circuit model

Thevenin equivalent circuit model(ECM) is a first-order RC circuit. The discharge direction is taken as the positive direction of current, as shown in the figure below.

![ECM](./imgs/Thevenin_equivalent_circuit.jpg)
<p align="center">Thevenin equivalent circuit</p>

The voltage on the polarization capacitor is denoted as Up. Then according to KVL and KCL we get the following equations.

```
UL(t) = UOC - Up(t) - Ro * I(t)          ......(1)
I(t) = Up(t) / Rp - Cp * (dUp(t)/dt)     ......(2)
```

The solution of the differential equation (2) is as follows.

```
Up(t) = C * exp(-t/tao) + I(t) * Rp      ......(3)
tao = Rp * Cp                            ......(4)
```
Here `C` is an arbitrary constant. The zero input response of the circuit model corresponds to the idle condition of the battery while zero state response corresponds to the working condition. The discretized form of Up in different states can be unified as follows.

```
Up(k+1) = Up(k) * exp(-Δt/tao) + 
          Rp * I(k) * (1 - exp(-Δt/tao)) ......(5)
```

The parameters in the Thevenin ECM, including `UOC`, `Ro`, `Rp` and `Cp`, are deemed to be related to the SoC of the battery. The relationships are usually identified through the so call Hybrid Pulse Power Characterization(HPPC) test.
