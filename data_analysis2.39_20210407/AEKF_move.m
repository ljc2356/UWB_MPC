clear all;
load('data/d_move_01.mat')

global result;
global m_result;
global mpc_result;
% 


%% 指定参数
% 运动噪声
% result(6,1).Delta_time(find(result(6,1).Delta_time ~= 0.079)) = 0.079;
Delta_u = 1;

% 观测噪声
% Delta_los_d = 0.0226;
% Delta_los_phi = 0.0046;   % 倒用噪声 倒用后期角度变化大 噪声影响小 因此多径权值比较大
% Delta_mpc_d = 0.0601;
% Delta_mpc_phi = 0.0080;

% Delta_los_d = 0.0226;
% Delta_los_phi = 0.0046;
% Delta_mpc_d = 0.0226;
% Delta_mpc_phi = 0.0046;  % 这一组噪声是设置是正用的结果 由于正用后期 角度变化小 噪声影响大 因此降低多径权值
% Delta_los_d = 0.0226;
% Delta_los_phi = 0.0046;   % 倒用噪声 倒用后期角度变化大 噪声影响小 因此多径权值比较大
% Delta_mpc_d = 0.0226;
% Delta_mpc_phi = 0.0046;
%%

antenna_num = 8;
index = antenna_num - 2;
% result(index,1).los_d.data(100:187,1) = 0;
% result(index,1).los_d.data(100:230,1) = 0;
m_result(index,1).antenna_num = antenna_num;
m_result(index,1).m(1,:) = [0.55 0 0.1 0 5 0.5]; % 指定初值
m_result(index,1).P{1} = eye(6)*100;  %相关噪声给大一些
m_result(index,1).Q{1} =  eye(4)/100000;   %运动噪声小一点
m_result(index,1).R{1} = eye(2)*100;     %观测噪声任取
m_result(index,1).e_flat(:,1) = zeros(2,1);
m_result(index,1).w_flat(:,1) = zeros(4,1);

NR = 2;
NQ = 2;
NR2 = 100;

mpc_result = m_result;
mpc_result(index,1).R{1} = [0.271650346223203,0;0,0.000429199642412428];

%% 开始进行LOS_EKF
real_index = 10000000;
for i = 2:useful_num
    %%  预测

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
        
        xy_minus = A*m_result(index,1).m(i-1,1:4)';
        P_xy_minus = A * m_result(index,1).P{i-1}(1:4,1:4) * A' + m_result(index,1).Q{i-1};
     %% Rk更新
     % 获得观测误差
         if ((result(index,1).los_d.data(i,1) == 0) || i >= real_index)  % 判断是否存在LOS  如果不存在
            m_result(index,1).m(i,1:4) = xy_minus';
            m_result(index,1).P{i}(1:4,1:4) = P_xy_minus;
            m_result(index,1).R{i} = m_result(index,1).R{i-1};
            m_result(index,1).e_flat(:,i) = m_result(index,1).e_flat(:,i-1);
            m_result(index,1).w_flat(:,i) = m_result(index,1).w_flat(:,i-1);
            m_result(index,1).Q{i} = m_result(index,1).Q{i-1} ;

            m_minus = A_expend * mpc_result(index,1).m(i-1,1:6)';
            P_m_minus = A_expend *  mpc_result(index,1).P{i-1}(1:6,1:6) * A_expend' + diag([diag(m_result(index,1).Q{i-1});0;0]);

         else

         zk_hat = [norm(xy_minus(1:2,1)');
                      atan2(xy_minus(2,1), xy_minus(1,1))];
         zk = [result(index,1).los_d.data(i,1);
                result(index,1).los_phi.data(i,1)];
         ek(1,1) = zk(1,1)- zk_hat(1,1);
         ek(2,1) = zk(2,1) - zk_hat(2,1);
         ek(2,1) = wrapToPi(ek(2,1));
         Hxk = Hx_los(xy_minus(1,1),xy_minus(2,1));

         a2 = (NR - 1)/NR;
         ek_flat = a2* m_result(index,1).e_flat(:,i-1) + (1/NR)*ek;
         Delta_Rk = (1/(NR-1))*(ek - ek_flat)*(ek - ek_flat)' - (1/NR)* Hxk * P_xy_minus * Hxk';
         Rk = abs(diag(diag(a2 * m_result(index,1).R{i-1} + Delta_Rk)));
         m_result(index,1).R{i} = Rk;
         m_result(index,1).e_flat(:,i) = ek_flat;
        
        Sk = Hxk*P_xy_minus*Hxk' + m_result(index,1).R{i};
        Kk = P_xy_minus*Hxk'*(Sk^(-1));
        xy_k = xy_minus + Kk*ek;
        P_xy_k = P_xy_minus - Kk * Hxk * P_xy_minus;
        %% 运动噪声协方差矩阵更新
        a1 = (NQ -1)/NQ;
        wk_hat = xy_k - xy_minus;
        wk_flat = a1 * m_result(index,1).w_flat(:,i-1) + (1/NQ)*wk_hat;
        Delta_Qk = (1/(NQ-1)) * (wk_hat - wk_flat)*(wk_hat - wk_flat)' + (1/NQ)*(P_xy_minus - A * m_result(index,1).P{i-1}(1:4,1:4) * A');
        Qk = abs(diag(diag(a1* m_result(index,1).Q{i-1} + Delta_Qk)));
        m_result(index,1).w_flat(:,i) = wk_flat;
        m_result(index,1).Q{i} = Qk;
        
        m_result(index,1).m(i,1:4) = xy_k';
        m_result(index,1).P{i}(1:4,1:4) = P_xy_k;
        
        %% 将首达径拷贝过来使用
        mpc_result(index,1).m(i,1:4)  = m_result(index,1).m(i,1:4) ;
        mpc_result(index,1).P{i}(1:4,1:4) = m_result(index,1).P{i}(1:4,1:4);
        
        %% 预测
        m_minus = [mpc_result(index,1).m(i,1:4)';mpc_result(index,1).m(i-1,5:6)'];  % 这里是如果有直达径的情况
        P_m_minus = [mpc_result(index,1).P{i}(1:4,1:4),mpc_result(index,1).P{i-1}(1:4,5:6);mpc_result(index,1).P{i-1}(5:6,1:4),mpc_result(index,1).P{i-1}(5:6,5:6)];
%         P_m_minus = [mpc_result(index,1).P{i}(1:4,1:4),zeros(4,2);zeros(2,4),mpc_result(index,1).P{i-1}(5:6,5:6)];        %% 自适应Rk
        P_m_minus = diag([mpc_result(index,1).P{i}(1,1),mpc_result(index,1).P{i}(2,2),mpc_result(index,1).P{i}(3,3),mpc_result(index,1).P{i}(4,4),mpc_result(index,1).P{i-1}(5,5),mpc_result(index,1).P{i-1}(6,6)]);        %% 自适应Rk
         end
        [xm,ym] = mirror(m_minus(1,1),m_minus(2,1),m_minus(5,1),m_minus(6,1));
        vk(1,1) = result(index,1).mpc_d.data(i,1) - norm([xm,ym]);
        vk(2,1) = result(index,1).mpc_phi.data(i,1) - atan2(ym,xm);
        vk(2,1) = wrapToPi(vk(2,1));
        Hxk = Hx_mirror(m_minus(1,1),m_minus(2,1),m_minus(5,1),m_minus(6,1));
        
        a2 = (NR2 -1)/NR2;
        ek_flat = a2 * mpc_result(index,1).e_flat(:,i-1) + (1/NR2)*vk;
        temp_ek = vk - ek_flat;
        temp_ek(2,1) = wrapToPi(temp_ek(2,1) );
        Delta_Rk = (1/(NR2 -1))*temp_ek*temp_ek' - 1/NR2 * Hxk * P_m_minus *Hxk';
        Rk = abs(diag(diag(a2 * mpc_result(index,1).R{i-1} + Delta_Rk)));
        mpc_result(index,1).R{i} = Rk;
        mpc_result(index,1).e_flat(:,i) = ek_flat;
        
        Sk = Hxk*P_m_minus*Hxk' + mpc_result(index,1).R{i} ;
        Kk = P_m_minus*Hxk'*(Sk^(-1));
        m_k = m_minus + Kk * vk;
        P_m_k = P_m_minus - Kk*Sk*Kk';
        mpc_result(index,1).m(i,1:6) = m_k';
        mpc_result(index,1).P{i}(1:6,1:6) = P_m_k;

end
run("Untitled2.m");











