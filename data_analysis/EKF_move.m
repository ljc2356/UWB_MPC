global result;
global m_result;
global mpc_result;

antenna_num = 8;
index = antenna_num - 2;
m_result(index,1).antenna_num = antenna_num;
m_result(index,1).m(1,:) = [0.6 0 0.5 0 3 0]; % 指定初值
m_result(index,1).P{1} = eye(6);

mpc_result = m_result;

% m参数的设置是 [ x y h theta];
% load('result_inv.mat')
% result(index,1).Delta_time(find(result(index,1).Delta_time ~= 0.079)) = 0.079;
%% 指定参数 
% 运动噪声
Delta_u = 0.008;
% 观测噪声
Delta_los_d = 0.0226;
Delta_los_phi = 0.0046;   % 倒用噪声 倒用后期角度变化大 噪声影响小 因此多径权值比较大
Delta_mpc_d = 0.06;
Delta_mpc_phi = 0.0080;

Delta_los_d = 0.0226;
Delta_los_phi = 0.0046;   % 倒用噪声 倒用后期角度变化大 噪声影响小 因此多径权值比较大
Delta_mpc_d = 0.0056;
Delta_mpc_phi = 0.0066;

Delta_u = 0.005;

% 观测噪声
Delta_los_d = 0.126;
Delta_los_phi = 0.056;   % 倒用噪声 倒用后期角度变化大 噪声影响小 因此多径权值比较大
Delta_mpc_d = 0.0601;
Delta_mpc_phi = 0.0080;

real_index = useful_num;

%% 开始进行LOS_EKF
% real_index = useful_num;
% real_index = 10;
for i = 2:useful_num
    % 预测
        
        Delta_time = result(index,1).Delta_time(i,1);
        A = [1 0 Delta_time 0;
            0 1 0 Delta_time;
            0 0 1 0;
            0 0 0 1];
        Qu = [ (Delta_u^2)*(Delta_time^3)/3  0  (Delta_u^2)*(Delta_time^2)/2  0;
               0  (Delta_u^2)*(Delta_time^3)/3  0  (Delta_u^2)*(Delta_time^2)/2;
               (Delta_u^2)*(Delta_time^2)/2  0  (Delta_u^2)*(Delta_time)  0;
               0  (Delta_u^2)*(Delta_time^2)/2  0  (Delta_u^2)*(Delta_time)];
        A_expend = [A,zeros(4,2);
                    zeros(2,4),eye(2)];
        Qu_expend = [Qu , zeros(4,2);
                    zeros(2,4),zeros(2,2)];
        
    if ((result(index,1).los_d.data(i,1) == 0) || i >= real_index)  % 判断是否存在LOS  如果不存在
        
        m_result(index,1).m(i,1:4) = (A * m_result(index,1).m(i-1,1:4)')';
        m_result(index,1).P{i}(1:4,1:4) = A * m_result(index,1).P{i-1}(1:4,1:4) * A' + Qu;
        

        m_minus = A_expend * mpc_result(index,1).m(i-1,1:6)';
        P_m_minus = A_expend *  mpc_result(index,1).P{i-1}(1:6,1:6) * A_expend' + Qu_expend;  
        
    else  %如果存在
       
        

        xy_minus = A*m_result(index,1).m(i-1,1:4)';
        P_xy_minus = A * m_result(index,1).P{i-1}(1:4,1:4) * A' + Qu;
        % 更新
        vk(1,1) = result(index,1).los_d.data(i,1) - norm(xy_minus(1:2,1)');
        vk(2,1) = result(index,1).los_phi.data(i,1) - atan2(xy_minus(2,1), xy_minus(1,1));
        vk(2,1) = wrapToPi(vk(2,1));
        Hxk = Hx_los(xy_minus(1,1),xy_minus(2,1));
        Sk = Hxk*P_xy_minus*Hxk' + [Delta_los_d^2, 0 ;0 , Delta_los_phi^2];
        Kk = P_xy_minus*Hxk'*(Sk^(-1));
        xy_k = xy_minus + Kk*vk;
        P_xy_k = P_xy_minus - Kk*Sk*Kk';

        m_result(index,1).m(i,1:4) = xy_k';
        m_result(index,1).P{i}(1:4,1:4) = P_xy_k;
        %% 拷贝过来 以供多径定位使用
        mpc_result(index,1).m(i,1:4)  = m_result(index,1).m(i,1:4) ;
        mpc_result(index,1).P{i}(1:4,1:4) = m_result(index,1).P{i}(1:4,1:4) ;
        %% 开始进行NLOS_EKF

        m_minus = [mpc_result(index,1).m(i,1:4)';mpc_result(index,1).m(i-1,5:6)'];  % 这里是如果有直达径的情况
       
        P_m_minus = [mpc_result(index,1).P{i}(1:4,1:4),mpc_result(index,1).P{i-1}(1:4,5:6);mpc_result(index,1).P{i-1}(5:6,1:4),mpc_result(index,1).P{i-1}(5:6,5:6)];
        P_m_minus = [mpc_result(index,1).P{i}(1:4,1:4),zeros(4,2);zeros(2,4),mpc_result(index,1).P{i-1}(5:6,5:6)];
    end
    
    if (result(index,1).mpc_d.data(i,1) == 0)  % 没有多径的时候则不进行更新
        mpc_result(index,1).m(i,1:6) = m_minus';
        mpc_result(index,1).P{i}(1:6,1:6) = P_m_minus;
    else
        [xm,ym] = mirror(m_minus(1,1),m_minus(2,1),m_minus(5,1),m_minus(6,1));
        vk(1,1) = result(index,1).mpc_d.data(i,1) - norm([xm,ym]);
        vk(2,1) = result(index,1).mpc_phi.data(i,1) - atan2(ym,xm);
        vk(2,1) = wrapToPi(vk(2,1));
        Hxk = Hx_mirror(m_minus(1,1),m_minus(2,1),m_minus(5,1),m_minus(6,1));
        Sk = Hxk*P_m_minus*Hxk' + diag([Delta_mpc_d^2, Delta_mpc_phi^2]);
        Kk = P_m_minus*Hxk'*(Sk^(-1));
        m_k = m_minus + Kk * vk;
        P_m_k = P_m_minus - Kk*Sk*Kk';
        mpc_result(index,1).m(i,1:6) = m_k';
        mpc_result(index,1).P{i}(1:6,1:6) = P_m_k;
    end
end
run("Untitled2.m");
% 
% 
% 
% 
% 
% 
% 
% 
% 



