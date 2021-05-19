%% IMU LOS MPC integrates federated Kalman filtering to locate the core script
% Range and Angle measurement data and accelerometer data are required
% After fusion, IMU_result Los_result Mpc_result is the same fusion result, saves the same exact location information, and is the output of this script
%% Load data and declare data
clear all;clc;close all;

load('data/20210413_indoor/Square.mat') %Load the Range and Angle measurement data
load('data/Acc_data/shape/Square_Acc.mat'); %Load the accelerometer data

global result; %Initializes the rangefinder data structure
global IMU_result; %Initialize the inertial navigation observation Kalman filter positioning result structure
global Los_result; %Initialize the direct path observation Kalman filter to locate the result structure
global Mpc_result; %Initialize the multi-path observation Kalman filter to locate the result structure

%% Initialize the initial values ​​and related parameters of the Kalman filter
antenna_num = 8;
index = antenna_num-2;
useful_num = length(result(index,1).los_d.data);
init_state = [3 -1 0 0 0 0 3.95 0]; %A relatively precise initial value helps in the fusion
init_P = [0.00001 0.00001 0.0001 0.0001 0.0001 0.0001 0.00001 0.00001]; %Confidence in an accurate initial value helps a lot

% Initialize the IMU derivative data
IMU_result.m(1,:) = init_state(1,1:6);
IMU_result.P{1} = diag(init_P(1:6));
IMU_result.Q{1} = eye(6)/10000;
IMU_result.R{1} = eye(2)/100;
IMU_result.e_flat(1,:) = zeros(1,2);
IMU_result.w_flat(1,:) = zeros(1,6);

% Initialize direct path filtering data
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
Mpc_result(index,1).R{1} = [eye(2)/100];% Observation noise is optional
Mpc_result(index,1).e_flat(1,:) = zeros(2,1)';
Mpc_result(index,1).w_flat(1,:) = zeros(6,1)';

% Given AEKF noise update parameters
NR = 100;
NQ = 100;


%% starts fusion positioning
 for i = 2: useful_num
    %% each sub-filter filtering
        % IMU filtering
        Delta_time = 0.079;
        A_a = [1 0 Delta_time 0 1/2*Delta_time^2 0;
               0 1 0 Delta_time 0 1/2*Delta_time^2;
               0 0 1 0 Delta_time 0;
               0 0 0 1 0 Delta_time;
               0 0 0 0 1 0;
               0 0 0 0 0 1];% uniform acceleration motion model
       Motion_model.fx = @(m,contrl) A_a*m;
       Motion_model.JFx = @(m,contrl) A_a;
       Obser_model.hx = @(m) m(5:6,1);
       Obser_model.JHx = @(m) [zeros(2,4),eye(2)];
       Observation = [Acceleration(i,1); Acceleration(i,2)];
       IMU_result = AEKF(IMU_result, i, Observation, Delta_time, Motion_model, Obser_model,[], NR,NQ); %Perform filtering
       
       % First arrival path observation filter
       Delta_time = 0.079;
       A = [1 0 Delta_time 0;
            0 1 0 Delta_time;
            0 0 1 0;
            0 0 0 1];% uniform motion model
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
       if (1)% is coupled?
           % Fusion Coefficient
           coef(1) = 1;
           coef(2) = 1;
           coef(3) = 1;
           
           P_LOS = Los_result(index,1).P{i}(1:4,1:4) * coef(1);
           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4) * coef(2);
           P_IMU = IMU_result.P{i}(1:4,1:4) * coef(3);

            Pg = (P_LOS^(-1) + P_MPC^(-1) + P_IMU^(-1) )^(-1);
            
            X1 = (P_LOS^(-1)) * Los_result(index,1).m(i,1:4)';
            X2 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
            X3 = (P_IMU^(-1)) * IMU_result.m(i,1:4)';
            Xg_hat = Pg * (X1 + X2 + X3);
            
            % Fusion estimation result output
            temp_los = Los_result(index,1).m(i,1:4)';% value before fusion
            Los_result(index,1).m(i,1:4) = Xg_hat';
            Mpc_result(index,1).m(i,1:4) = Xg_hat';
            IMU_result.m(i,1:4) = Xg_hat';
            
            
            coef_sum = 1/coef(1) + 1/coef(2) + 1/coef(3);
            P_coef(1) = coef(1) * coef_sum;
            P_coef(2) = coef(2) * coef_sum;
            P_coef(3) = coef(3) * coef_sum;
            
            % Fusion estimated covariance output
           Los_result(index,1).P{i}(1:4,1:4) = P_coef(1) * Pg;
           Mpc_result(index,1).P{i}(1:4,1:4) = P_coef(2) * Pg;
           IMU_result.P{i}(1:4,1:4) = P_coef(3) * Pg;
       %% Update the motion noise covariance according to the fusion result
           if (norm(temp_los-Xg_hat(1:4,1))<=0.1)% Only perform fusion when the direct path is relatively good
           
           a1 = (NQ -1)/NQ;
           m_minus = A_a * IMU_result.m(i-1,1:6)';
           P_minus = A_a * IMU_result.P{i-1}(1:6,1:6) * A_a' + IMU_result.Q{i-1}(1:6,1:6);% where Q{i- 1) The matrix is ​​the result of the fusion
           
           wk_hat = [Xg_hat;IMU_result.m(i,5:6)']-m_minus;
           wk_flat = a1 * IMU_result.w_flat(i-1,:)' + 1/NQ * wk_hat;% here Los.w_flat(i-1,:) is also the result of fusion
           Delta_Qk = 1/(NQ-1) * (wk_hat-wk_flat)*(wk_hat-wk_flat)' + 1/NQ * (P_minus-A_a * IMU_result.P{i-1}(1:6,1:6) * A_a');
           Qk = abs(diag(diag(a1 * IMU_result.Q{i-1}(1:6,1:6) + Delta_Qk)));
           
           Los_result(index,1).w_flat(i,1:4) = wk_flat(1:4,1)';
           Los_result(index,1).Q{i}(1:4,1:4) = Qk(1:4,1:4);
           Mpc_result(index,1).w_flat(i,1:4) = wk_flat(1:4,1)';
           Mpc_result(index,1).Q{i}(1:4,1:4) = Qk(1:4,1:4);
           IMU_result.w_flat(i,1:6) = wk_flat(1:6,1)';
           IMU_result.Q{i}(1:6,1:6) = Qk(1:6,1:6);
           end
           
           
       end
 end
% save("data/Eight antenna positioning result/move_04 triangle/IMU_result.mat","IMU_result");
run("DrawOneFast.m");