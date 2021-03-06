clear all;clc;close all;
load('data/20210413_indoor/move_07.mat')


global result;

global Los_result;
global Mpc_result;
global Avg_result;
beta = 1/2;

%% 指定参数
Delta_u = 0.01;

%%

antenna_num = 8;
index = antenna_num - 2;
init_state = [3      -1        0     0.1    0    0      4         0];
init_P =     [0.00001   0.00001    0.0001  0.0001  0.0001  0.0001  0.00001  0.00001];
% init_P =     [0.0001   0.0001    0.001  0.001  0.001  0.001  0.0001  0.0001];
Los_result(index,1).antenna_num = antenna_num;
Los_result(index,1).m(1,:) = init_state(1,1:4);% 指定初值
Los_result(index,1).P{1} = diag(init_P(1:4));  %相关噪声给大一些
Los_result(index,1).Q{1} = eye(4)/10000;  %运动噪声小一点
Los_result(index,1).R{1} = eye(2)/100;    %观测噪声任取
Los_result(index,1).e_flat(1,:) = zeros(2,1)';
Los_result(index,1).w_flat(1,:) = zeros(4,1)';
Los_result(index,1).error_index(1,1) = 0;

Mpc_result(index,1).antenna_num = antenna_num;
Mpc_result(index,1).m(1,:) = [init_state(1,1:4),init_state(1,7:8)];% 指定初值
Mpc_result(index,1).P{1} =  diag([init_P(1:4),init_P(7:8)]); %相关噪声给大一些
Mpc_result(index,1).Q{1} = [eye(4)/10000,zeros(4,2);zeros(2,4),zeros(2,2)];   %运动噪声小一点
Mpc_result(index,1).R{1} = [eye(2)/100]; %观测噪声任取
Mpc_result(index,1).e_flat(1,:) = zeros(2,1)';
Mpc_result(index,1).w_flat(1,:) = zeros(6,1)';

NR = 10;
NQ = 10;

%% 开始进行LOS_EKF
real_index = 10000000;
useful_num = length(result(index,1).los_d.data);

 for i = 2:useful_num
    %%  预测
       
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
       Observation =   [result(index,1).los_d.data(i,1);
                         result(index,1).los_phi.data(i,1)];
       
           Los_result(index,1) = AEKF_X(Los_result(index,1) , i , Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ,10);
%         Los_result(index,1) = AEKF(Los_result(index,1) , i , Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ);
%         Los_result(index,1).error_index(i,1) = 0;
        A_expend = [1 0 Delta_time 0       0  0 ;
                   0 1     0  Delta_time  0  0 ;
                   0 0     1      0       0  0 ;
                   0 0     0      1       0  0 ;
                   0 0     0      0       1  0 ;
                   0 0     0      0       0  1 ];
       Motion_model.fx = @(m,contrl) A_expend*m;
       Motion_model.JFx = @(m,contrl) A_expend;
       Obser_model.hx = @h_mirror;
       Obser_model.JHx = @Hx_J_mirror;
       Observation =   [result(index,1).mpc_d.data(i,1);
                        result(index,1).mpc_phi.data(i,1)];
       Mpc_result(index,1) = AEKF(Mpc_result(index,1) , i , Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ);

       %% 运动状态协方差应该公用
       if ( 1 )   %是否耦合
   
       
       %% 进行融合
    if (Los_result(index,1).error_index(i,1) == 1)
           
           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4);

           Pg = (  P_MPC^(-1))^(-1);
           X3 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
           Xg_hat = Pg * (X3);

           Los_result(index,1).m(i,1:4) = Xg_hat';
           Mpc_result(index,1).m(i,1:4) = Xg_hat';


           Mpc_result(index,1).P{i}(1:4,1:4)  = 1* Pg;
%            Los_result(index,1).P{i}(1:4,1:4) = 2*Pg;
%            
           
           Avg_result.m(i,:) = Xg_hat';
           Avg_result.P{i} = Pg;
           
    else
           Mpc_result(index,1).Q{i}(1:4,1:4) = Los_result(index,1).Q{i}(1:4,1:4) ;
           
           coef(1) = 1;
           coef(2) = 1;
           P_LOS = Los_result(index,1).P{i}(1:4,1:4) * coef(1);
           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4) * coef(2);

           Pg = ( P_LOS^(-1) +  P_MPC^(-1))^(-1);
           X2 = (P_LOS^(-1)) * Los_result(index,1).m(i,1:4)';
           X3 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
           Xg_hat = Pg * (X2 + X3);
           
           Los_result(index,1).m(i,1:4) = Xg_hat';
           Mpc_result(index,1).m(i,1:4) = Xg_hat';
           
           coef_sum = 1/coef(1) + 1/coef(2);
           P_coef(1) = coef(1) * coef_sum;
           P_coef(2) = coef(2) * coef_sum;
           

           Los_result(index,1).P{i}(1:4,1:4) = P_coef(1) * Pg;
           Mpc_result(index,1).P{i}(1:4,1:4)  = P_coef(2)  *  Pg;

           Avg_result.m(i,:) = Xg_hat';
           Avg_result.P{i} = Pg;
    end

       end
 end
 save("Mpc_result.mat","Mpc_result");





%% 开始不融合

 for i = 2:useful_num
    %%  预测
       
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
       Observation =   [result(index,1).los_d.data(i,1);
                         result(index,1).los_phi.data(i,1)];
       
%           Los_result(index,1) = AEKF_X(Los_result(index,1) , i , Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ,10);
        Los_result(index,1) = AEKF(Los_result(index,1) , i , Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ);
        Los_result(index,1).error_index(i,1) = 0;
        A_expend = [1 0 Delta_time 0       0  0 ;
                   0 1     0  Delta_time  0  0 ;
                   0 0     1      0       0  0 ;
                   0 0     0      1       0  0 ;
                   0 0     0      0       1  0 ;
                   0 0     0      0       0  1 ];
       Motion_model.fx = @(m,contrl) A_expend*m;
       Motion_model.JFx = @(m,contrl) A_expend;
       Obser_model.hx = @h_mirror;
       Obser_model.JHx = @Hx_J_mirror;
       Observation =   [result(index,1).mpc_d.data(i,1);
                        result(index,1).mpc_phi.data(i,1)];
       Mpc_result(index,1) = AEKF(Mpc_result(index,1) , i , Observation, Delta_time, Motion_model, Obser_model,[2], NR,NQ);

       %% 运动状态协方差应该公用
       if ( 0 )   %是否耦合
   
       
       %% 进行融合
    if (Los_result(index,1).error_index(i,1) == 1)

           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4);

           Pg = (  P_MPC^(-1))^(-1);
           X3 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
           Xg_hat = Pg * (X3);

           Los_result(index,1).m(i,1:4) = Xg_hat';
           Mpc_result(index,1).m(i,1:4) = Xg_hat';


           Mpc_result(index,1).P{i}(1:4,1:4)  = 1* Pg;
%            Los_result(index,1).P{i}(1:4,1:4) = 2*Pg;
%            
           
           Avg_result.m(i,:) = Xg_hat';
           Avg_result.P{i} = Pg;
           
    else
                  Mpc_result(index,1).Q{i}(1:4,1:4) = Los_result(index,1).Q{i}(1:4,1:4) ;
           
           coef(1) = 1;
           coef(2) = 1.3;
           P_LOS = Los_result(index,1).P{i}(1:4,1:4) * coef(1);
           P_MPC = Mpc_result(index,1).P{i}(1:4,1:4) * coef(2);

           Pg = ( P_LOS^(-1) +  P_MPC^(-1))^(-1);
           X2 = (P_LOS^(-1)) * Los_result(index,1).m(i,1:4)';
           X3 = (P_MPC^(-1)) * Mpc_result(index,1).m(i,1:4)';
           Xg_hat = Pg * (X2 + X3);
           
           Los_result(index,1).m(i,1:4) = Xg_hat';
           Mpc_result(index,1).m(i,1:4) = Xg_hat';
           
           coef_sum = 1/coef(1) + 1/coef(2);
           P_coef(1) = coef(1) * coef_sum;
           P_coef(2) = coef(2) * coef_sum;
           

           Los_result(index,1).P{i}(1:4,1:4) = P_coef(1) * Pg;
           Mpc_result(index,1).P{i}(1:4,1:4)  = P_coef(2) *  Pg;

           Avg_result.m(i,:) = Xg_hat';
           Avg_result.P{i} = Pg;
    end

       end
 end
 save("Los_result.mat","Los_result");
%  run("DrawTwo.m");








