%% LOS MPC fusion Federated Kalman filter positioning core script (no inertial navigation data fusion)
% Need the main.m ranging angle measurement data output from the range measurement algorithm script
% After fusion, Los_result Mpc_result is the same fusion result, saves the same precise location information, which are all of this script
%% Load data and declare data
clear all;clc;close all;
load('data/20210413_indoor/Square.mat')

global result; %Initialize the distance measurement and teaching data structure
global Los_result; %Initialize direct path observation Kalman filter positioning result structure
global Mpc_result; %Initialize multipath observation Kalman filter positioning result structure
 

%% initialize Kalman filter initial value and related parameters

antenna_num = 8;
index = antenna_num-2;
init_state = [3 -1 0 0 0 0 4 0];% Give a relatively accurate initial value to help fusion
init_P = [0.00001 0.00001 0.0001 0.0001 0.0001 0.0001 0.00001 0.00001];% is confident in the accurate initial value, which is helpful for integration

% Initialize direct path filter data
Los_result(index,1).antenna_num = antenna_num;
Los_result(index,1).m(1,:) = init_state(1,1:4);% specify the initial value
Los_result(index,1).P{1} = diag(init_P(1:4));% relative noise is bigger
Los_result(index,1).Q{1} = eye(4)/10000;% motion noise is smaller
Los_result(index,1).R{1} = eye(2)/100;% Observation noise is optional
Los_result(index,1).e_flat(1,:) = zeros(2,1)';
Los_result(index,1).w_flat(1,:) = zeros(4,1)';
Los_result(index,1).error_index(1,1) = 0;
% Initialize multipath filtering data
Mpc_result(index,1).antenna_num = antenna_num;
Mpc_result(index,1).m(1,:) = [init_state(1,1:4),init_state(1,7:8)];% specify the initial value
Mpc_result(index,1).P{1} = diag([init_P(1:4),init_P(7:8)]);% relative noise is bigger
Mpc_result(index,1).Q{1} = [eye(4)/10000,zeros(4,2);zeros(2,4),zeros(2,2)];% motion noise is smaller
Mpc_result(index,1).R{1} = eye(2)/100;% Observation noise is optional
Mpc_result(index,1).e_flat(1,:) = zeros(2,1)';
Mpc_result(index,1).w_flat(1,:) = zeros(6,1)';
% Given AEKF noise update parameters
NR = 100;
NQ = 100;

%% started LOS_EKF
useful_num = length(result(index,1).los_d.data);
 for i = 2: useful_num
    %% each sub-filter filtering
    
       % First arrival path observation filter
       Delta_time = 0.079;
       A = [1 0 Delta_time 0;
            0 1 0 Delta_time;
            0 0 1 0;
            0 0 0 1];
       Motion_model.fx = @(m,contrl) A*m;
       Motion_model.JFx = @(m,contrl) A;
       Obser_model.hx = @(m) [norm(m(1:2,1)'); atan2(m(2,1), m(1,1))];
       Obser_model.JHx = @Hx_J_los;
       Observation = [result(index,1).los_d.data(i,1);
                         result(index,1).los_phi.data(i,1)];
       
       Los_result(index,1) = AEKF_X(Los_result(index,1), i, Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ,10);

        % Multipath observation filter
        A_expend = [1 0 Delta_time 0 0 0;
                   0 1 0 Delta_time 0 0;
                   0 0 1 0 0 0;
                   0 0 0 1 0 0;
                   0 0 0 0 1 0;
                   0 0 0 0 0 1 ];
       Motion_model.fx = @(m,contrl) A_expend*m;
       Motion_model.JFx = @(m,contrl) A_expend;
       Obser_model.hx = @h_mirror;
       Obser_model.JHx = @Hx_J_mirror;
       Observation = [result(index,1).mpc_d.data(i,1);
                        result(index,1).mpc_phi.data(i,1)];
       Mpc_result(index,1) = AEKF(Mpc_result(index,1), i, Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ);

      %% federated filter fusion
       if (1)% is coupled
   
                
           coef(1) = 1;
           coef(2) = 1;
           
           P_LOS = Los_result(index,1).P{i}(1:4,1:4) * coef(1);
           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4) * coef(2);

           Pg = (P_LOS^(-1) + P_MPC^(-1))^(-1);
           X2 = (P_LOS^(-1)) * Los_result(index,1).m(i,1:4)';
           X3 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
           Xg_hat = Pg * (X2 + X3);
           
           temp_los = Los_result(index,1).m(i,1:4)';% value before fusion
           Los_result(index,1).m(i,1:4) = Xg_hat';
           Mpc_result(index,1).m(i,1:4) = Xg_hat';
           
           coef_sum = 1/coef(1) + 1/coef(2);
           P_coef(1) = coef(1) * coef_sum;
           P_coef(2) = coef(2) * coef_sum;

           Los_result(index,1).P{i}(1:4,1:4) = P_coef(1) * Pg;
           Mpc_result(index,1).P{i}(1:4,1:4) = P_coef(2) * Pg;
             
           %% does not update the motion noise matrix based on the results of each sub-filter, but updates based on the results of the fusion
           if (norm(temp_los-Xg_hat(1:4,1))<=0.1)% Only perform fusion when the direct path is relatively good
               a1 = (NQ -1)/NQ;
               m_minus = A * Los_result(index,1).m(i-1,1:4)';
               P_minus = A * Los_result(index,1).P{i-1}(1:4,1:4) * A'+ Los_result(index,1).Q{i-1}(1:4,1: 4);% The Q{i-1} matrix here is the result of fusion

               wk_hat = Xg_hat-m_minus;
               wk_flat = a1 * Los_result(index,1).w_flat(i-1,:)' + 1/NQ * wk_hat;% Here Los.w_flat(i-1,:) is also the result of fusion
               Delta_Qk = 1/(NQ-1) * (wk_hat-wk_flat)*(wk_hat-wk_flat)' + 1/NQ * (P_minus-A * Los_result(index,1).P{i-1}(1:4, 1:4) * A');
               Qk = abs(diag(diag(a1 * Los_result(index,1).Q{i-1}(1:4,1:4) + Delta_Qk)));
               Los_result(index,1).w_flat(i,1:4) = wk_flat';
               Los_result(index,1).Q{i}(1:4,1:4) = Qk;
               Mpc_result(index,1).w_flat(i,1:4) = wk_flat';
               Mpc_result(index,1).Q{i}(1:4,1:4) = Qk;
           end
           
       end
 end
 save("Mpc_result.mat","Mpc_result");

%% Recalculate the results without fusion for comparison

 for i = 2: useful_num
    %% forecast
       
       Delta_time = result(index,1).Delta_time(i,1);
       Delta_time = 0.079;
       A = [1 0 Delta_time 0;
            0 1 0 Delta_time;
            0 0 1 0;
            0 0 0 1];
       Motion_model.fx = @(m,contrl) A*m;
       Motion_model.JFx = @(m,contrl) A;
       Obser_model.hx = @(m) [norm(m(1:2,1)'); atan2(m(2,1), m(1,1))];
       Obser_model.JHx = @Hx_J_los;
       Observation = [result(index,1).los_d.data(i,1);
                         result(index,1).los_phi.data(i,1)];
       
% Los_result(index,1) = AEKF_X(Los_result(index,1), i, Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ,10);
        Los_result(index,1) = AEKF(Los_result(index,1), i, Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ);
        Los_result(index,1).error_index(i,1) = 0;
        A_expend = [1 0 Delta_time 0 0 0;
                   0 1 0 Delta_time 0 0;
                   0 0 1 0 0 0;
                   0 0 0 1 0 0;
                   0 0 0 0 1 0;
                   0 0 0 0 0 1 ];
       Motion_model.fx = @(m,contrl) A_expend*m;
       Motion_model.JFx = @(m,contrl) A_expend;
       Obser_model.hx = @h_mirror;
       Obser_model.JHx = @Hx_J_mirror;
       Observation = [result(index,1).mpc_d.data(i,1);
                        result(index,1).mpc_phi.data(i,1)];
       Mpc_result(index,1) = AEKF(Mpc_result(index,1), i, Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ);

       %% sports state covariance should be shared
       if (0)% is coupled
   
       
       %% for fusion
    if (Los_result(index,1).error_index(i,1) == 1)

           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4);

           Pg = (P_MPC^(-1))^(-1);
           X3 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
           Xg_hat = Pg * (X3);

           Los_result(index,1).m(i,1:4) = Xg_hat';
           Mpc_result(index,1).m(i,1:4) = Xg_hat';


           Mpc_result(index,1).P{i}(1:4,1:4) = 1* Pg;
% Los_result(index,1).P{i}(1:4,1:4) = 2*Pg;
%
           
           Avg_result.m(i,:) = Xg_hat';
           Avg_result.P{i} = Pg;
           
    else
                  Mpc_result(index,1).Q{i}(1:4,1:4) = Los_result(index,1).Q{i}(1:4,1:4);
           
           coef(1) = 1;
           coef(2) = 1.3;
           P_LOS = Los_result(index,1).P{i}(1:4,1:4) * coef(1);
           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4) * coef(2);

           Pg = (P_LOS^(-1) + P_MPC^(-1))^(-1);
           X2 = (P_LOS^(-1)) * Los_result(index,1).m(i,1:4)';
           X3 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
           Xg_hat = Pg * (X2 + X3);
           
           Los_result(index,1).m(i,1:4) = Xg_hat';
           Mpc_result(index,1).m(i,1:4) = Xg_hat';
           
           coef_sum = 1/coef(1) + 1/coef(2);
           P_coef(1) = coef(1) * coef_sum;
           P_coef(2) = coef(2) * coef_sum;
           

           Los_result(index,1).P{i}(1:4,1:4) = P_coef(1) * Pg;
           Mpc_result(index,1).P{i}(1:4,1:4) = P_coef(2) * Pg;

           Avg_result.m(i,:) = Xg_hat';
           Avg_result.P{i} = Pg;
    end
       end
 end
 save("Los_result.mat","Los_result");
 %% draw a comparison chart
 run("DrawTwoFast.m");



