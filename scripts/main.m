%% ----------------------------
% Input: SOC_est_init: The initial value of estimated SOC
%        Work_mode: Mode of working condition 1 --> BBDST, 2 --> constant current
%% ----------------------------
function main(SOC_est_init, Work_mode)
    tic;  % 计算程序运行时间，开始
    if nargin == 0  % Set parameter by default
        SoC_est_init = 1;
        Work_mode = 1;
    elseif nargin == 1
        SoC_est_init = 1;
    end
    if Work_mode == 1
        sim BBDST_workingcondition;
        I = -(current.data)'*1.5/50;
    elseif Work_mode == 2
        N = 60001;
        I=1.5*ones(1,N);
        I(ceil(N/4):ceil(N/3)) = 0;
        I(ceil(N*2/3):ceil(N*3/4)) = 0;
    else
        disp("输入参数错误！");
        disp("SOC_est_init : The initial value of estimated SOC");
        disp("Work_mode: Mode of working condition");
        disp("                       1 --> BBDST, 2 --> constant current ");
        return;
    end
    [mean_err_EKF,standard_err_EKF,mean_err_UKF,standard_err_UKF]=EKF_UKF_Thev(SOC_est_init, I);
    fprintf("mean_err_EKF --> %f\n", mean_err_EKF);
    fprintf("standard_err_EKF --> %f\n", standard_err_EKF);
    fprintf("mean_err_UKF --> %f\n", mean_err_UKF);
    fprintf("standard_err_UKF --> %f\n", standard_err_UKF);
    toc;  % 计算程序运行时间，结束
end