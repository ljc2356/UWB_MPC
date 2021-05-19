%% EKF two-stage fusion algorithm core algorithm script
% Enter    the distance and angle data in the data folder
% Output
% m_result   structure of Direct path EKF positioning result
% mpc_result structure of Two-segment fusion EKF positioning result
%%
clear all;close all;clc;
global result;
global m_result;
global mpc_result;
load('data/20210407_indoor/L.mat')

%Initialize the relevant structure
antenna_num = 8;
index = antenna_num-2;
m_result(index,1).antenna_num = antenna_num;
m_result(index,1).m(1,:) = [3 -1 0 0.5 4 0];% Specify the initial value
m_result(index,1).P{1} = eye(6);
mpc_result = m_result;
useful_num = length(result(index,1).los_d.data);


%% specifies the parameter
% Motion noise parameters
Delta_u = 0.005;

% Observed noise parameters
Delta_los_d = 0.126;
Delta_los_phi = 0.056;
Delta_mpc_d = 0.0601;
Delta_mpc_phi = 0.0080;

%% started LOS_EKF

for i = 2: useful_num
    % LOS forecast
        Delta_time = result(index,1).Delta_time(i,1);
        A = [1 0 Delta_time 0;
            0 1 0 Delta_time;
            0 0 1 0;
            0 0 0 1];
        Qu = [(Delta_u^2)*(Delta_time^3)/3 0 (Delta_u^2)*(Delta_time^2)/2 0;
               0 (Delta_u^2)*(Delta_time^3)/3 0 (Delta_u^2)*(Delta_time^2)/2;
               (Delta_u^2)*(Delta_time^2)/2 0 (Delta_u^2)*(Delta_time) 0;
               0 (Delta_u^2)*(Delta_time^2)/2 0 (Delta_u^2)*(Delta_time)];
        A_expend = [A,zeros(4,2);
                    zeros(2,4),eye(2)];
        Qu_expend = [Qu, zeros(4,2);
                    zeros(2,4),zeros(2,2)];
        
    if ((result(index,1).los_d.data(i,1) == 0))% Determine whether there is LOS data
        % If it does not exist
        m_result(index,1).m(i,1:4) = (A * m_result(index,1).m(i-1,1:4)')';
        m_result(index,1).P{i}(1:4,1:4) = A * m_result(index,1).P{i-1}(1:4,1:4) * A'+ Qu ;
        

        m_minus = A_expend * mpc_result(index,1).m(i-1,1:6)';
        P_m_minus = A_expend * mpc_result(index,1).P{i-1}(1:6,1:6) * A_expend' + Qu_expend;
        
    else% if it exists
       
        

        xy_minus = A*m_result(index,1).m(i-1,1:4)';
        P_xy_minus = A * m_result(index,1).P{i-1}(1:4,1:4) * A'+ Qu;
        % Update status with los data
        vk(1,1) = result(index,1).los_d.data(i,1)-norm(xy_minus(1:2,1)');
        vk(2,1) = result(index,1).los_phi.data(i,1)-atan2(xy_minus(2,1), xy_minus(1,1));
        vk(2,1) = wrapToPi(vk(2,1));
        Hxk = Hx_J_los([xy_minus(1,1);xy_minus(2,1)]);
        Sk = Hxk*P_xy_minus*Hxk' + [Delta_los_d^2, 0 ;0, Delta_los_phi^2];
        Kk = P_xy_minus*Hxk'*(Sk^(-1));
        xy_k = xy_minus + Kk*vk;
        P_xy_k = P_xy_minus-Kk*Sk*Kk';

        m_result(index,1).m(i,1:4) = xy_k';
        m_result(index,1).P{i}(1:4,1:4) = P_xy_k;
        %% copied over for multi-path positioning
        mpc_result(index,1).m(i,1:4) = m_result(index,1).m(i,1:4);
        mpc_result(index,1).P{i}(1:4,1:4) = m_result(index,1).P{i}(1:4,1:4);
        %% started NLOS_EKF

        m_minus = [mpc_result(index,1).m(i,1:4)';mpc_result(index,1).m(i-1,5:6)'];% here is if there is a direct path
        P_m_minus = [mpc_result(index,1).P{i}(1:4,1:4),zeros(4,2);zeros(2,4),mpc_result(index,1).P{i-1 }(5:6,5:6)];
    end
    
    if (result(index,1).mpc_d.data(i,1) == 0)% do not update when there is no multipath
        mpc_result(index,1).m(i,1:6) = m_minus';
        mpc_result(index,1).P{i}(1:6,1:6) = P_m_minus;
    else
        [xm,ym] = mirror(m_minus(1,1),m_minus(2,1),m_minus(5,1),m_minus(6,1));
        vk(1,1) = result(index,1).mpc_d.data(i,1)-norm([xm,ym]);
        vk(2,1) = result(index,1).mpc_phi.data(i,1)-atan2(ym,xm);
        vk(2,1) = wrapToPi(vk(2,1));
        Hxk = Hx_J_mirror(m_minus);
        Sk = Hxk*P_m_minus*Hxk' + diag([Delta_mpc_d^2, Delta_mpc_phi^2]);
        Kk = P_m_minus*Hxk'*(Sk^(-1));
        m_k = m_minus + Kk * vk;
        P_m_k = P_m_minus-Kk*Sk*Kk';
        mpc_result(index,1).m(i,1:6) = m_k';
        mpc_result(index,1).P{i}(1:6,1:6) = P_m_k;
    end
end

xlabel('x');
ylabel('y');
set(gca,'FontSize',12);
hd(1) = scatter(m_result(index,1).m(:,1),m_result(index,1).m(:,2),50,"ro");
hold on;
hd(2) = scatter(mpc_result(index,1).m(:,1),mpc_result(index,1).m(:,2),50,"b*");



