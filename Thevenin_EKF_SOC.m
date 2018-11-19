%-----------------------------------------------
%文 件 名：EKF_SOC
%文件描述：基于扩展卡尔曼 锂电池Thevenin模型 SOC估算
%时    间：Nov.6.2018
%编 辑 者：Wanglu
%-------------------Detlab------------------
function Thevenin_EKF_SOC
close all;
clc; 
tic; 
%% -------------初始化参数-------------------
X=[1;0];                    %标准系统初值[SOC;极化电压]
P0=[0.1 0;0 0.1];           %状态误差协方差初值 

Q = 1e-5;                    %过程噪声
R = 5e-4;                    %观测噪声

UL(1,1)=0;
i=50;                       %放电电流，恒流
b=0.98;                     %放电效率
Q0=50*3600;                 %电池总容量
t_smpl=1;                   %采样时间，2s
N=3600/t_smpl;              %假定1个小时放完 

error(1,1)=0;
%% --------------模拟实际值----------
for t=2:N 
    SOC = X(1,t-1);
    R_int = 0.00573 - 0.03427*SOC^1 + 0.1455*SOC^2 - 0.32647*SOC^3 ...
            + 0.41465*SOC^4 - 0.28992*SOC^5 + 0.09353*SOC^6 - 0.00634*SOC^7;                   %欧姆内阻
    Rp = 0.01513 - 0.18008*SOC^1 + 1.05147*SOC^2 - 3.27616*SOC^3 ...
         + 5.79793*SOC^4 - 5.81819*SOC^5 + 3.08032*SOC^6 - 0.66827*SOC^7;                      %极化电容
    Cp = 47718.90713 - 1.00583E6*SOC^1 + 9.2653E6*SOC^2 - 3.91088E7*SOC^3 ...
        + 8.85892E7*SOC^4 - 1.11014E8*SOC^5 + 7.22811E7*SOC^6 - 1.90336E7*SOC^7;               %极化电阻
    Tao = Rp*Cp;                   %时间常数，"τ"
    % --------------A矩阵，状态转移----------
    A1 = 1;
    A2 = exp(-t_smpl/Tao);
    A = [A1 0;0 A2];
    % --------------B矩阵，输入控制----------
    B1 = -t_smpl*b/Q0;
    B2 = Rp*(1-exp(-t_smpl/Tao));
    B = [B1;B2];
   
    X(:,t)=A*X(:,t-1)+B*i+sqrt(Q)*randn(2,1);           %状态方程,真实值模拟   
    SOC = X(1,t);
    Uoc= -23.60229*SOC^7 + 141.34077*SOC^6 - 314.92282*SOC^5 ...
              + 345.34531*SOC^4 - 200.15462*SOC^3 ...
              + 60.21383*SOC^2 - 7.88447*SOC+3.2377;    %拟合开路电压与SOC关系方程
    Up = X(2,t);    
    UL(1,t)=Uoc-Up-i*R_int+sqrt(R)*randn;      %模拟实际电压电流
end
%% ---------------Kalman Filter 算法------------------
X_update=[0.9;0];
X_predict=[0;0];
for t=2:N
    
    SOC = X_update(1,t-1);
    R_int = 0.00573 - 0.03427*SOC^1 + 0.1455*SOC^2 - 0.32647*SOC^3 ...
            + 0.41465*SOC^4 - 0.28992*SOC^5 + 0.09353*SOC^6 - 0.00634*SOC^7;                   %欧姆内阻
    Rp = 0.01513 - 0.18008*SOC^1 + 1.05147*SOC^2 - 3.27616*SOC^3 ...
         + 5.79793*SOC^4 - 5.81819*SOC^5 + 3.08032*SOC^6 - 0.66827*SOC^7;                      %极化电容
    Cp = 47718.90713 - 1.00583E6*SOC^1 + 9.2653E6*SOC^2 - 3.91088E7*SOC^3 ...
        + 8.85892E7*SOC^4 - 1.11014E8*SOC^5 + 7.22811E7*SOC^6 - 1.90336E7*SOC^7;               %极化电阻
    Tao = Rp*Cp;                   %时间常数-τ
    % --------------A矩阵，状态转移----------
    A1 = 1;
    A2 = exp(-t_smpl/Tao);
    A = [A1 0;0 A2];
    % --------------B矩阵，输入控制----------
    B1 = -t_smpl*b/Q0;
    B2 = Rp*(1-exp(-t_smpl/Tao));
    B = [B1;B2];
%% -------------预测----------------    
    X_predict(:,t)=A*X_update(:,t-1)+B*i;         %由上一时刻状态量预测该时刻状态量，包括SOC,极化电压值 
    SOC_predict = X_predict(1,t);
    Up_predict = X_predict(2,t);
    Uoc= -23.60229*SOC_predict^7 + 141.34077*SOC_predict^6 - 314.92282*SOC_predict^5 ...
              + 345.34531*SOC_predict^4 - 200.15462*SOC_predict^3 ...
              + 60.21383*SOC_predict^2 - 7.88447*SOC_predict+3.2377173;
   VL=Uoc-Up_predict-i*R_int;       %预测电压电流 

   P1=A*P0*A'+ [Q 0;0 Q];                               %方差
    %% --------------C矩阵-------------
    C1=-23.60229*7*SOC_predict^6 + 141.34077*6*SOC_predict^5 - 314.92282*5*SOC_predict^4 ...
              + 345.34531*4*SOC_predict^3 - 200.15462*3*SOC_predict^2 ...
              + 60.21383*2*SOC_predict - 7.88447;
    C=[C1 -1];
    %% --------------更新--------------     
    K=P1*C'/(C*P1*C'+ R);                                  %增益

    X_update(:,t)=X_predict(:,t)+K*(UL(1,t)-VL);     %得到估计值

    P0=P1-K*C*P1;
    
    error(:,t)=X_update(1,t)-X(1,t);                   %滤波处理后的误差
end 

%% ---------------画图---------------
t=1:N;

figure;
subplot(2,1,1)
plot(t,X_update(1,:),'.',t,X(1,:),'g');
grid on;
legend('预测值','真实值');
xlabel('t(s)');
ylabel('SOC/100%');

subplot(2,1,2)
plot(t,error(1,:));
grid on;
legend('EKalman Filter-error');
xlabel(' t(s)');
ylabel('error/100%');

toc;      %计算仿真程序运行时间
end


