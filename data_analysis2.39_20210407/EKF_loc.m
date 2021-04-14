global result;
global m_result;
global mpc_result;


antenna_num = 8;
index = antenna_num - 2;
m_result(index,1).antenna_num = antenna_num;
m_result(index,1).m(1,:) = [1 -1 3 0]; % 指定初值
m_result(index,1).P{1} = eye(4);

mpc_result = m_result;
% m参数的设置是 [ x y h theta];

%% 指定参数
% 运动噪声
Delta_x = 0.2;
Delta_y = 0.2;
% 观测噪声
Delta_los_d = 0.002;
Delta_los_phi = 0.05;
Delta_mpc_d = 0.06;
Delta_mpc_phi = 0.05;

%% 开始进行LOS_EKF
for i = 2:useful_num
    % 预测
    if (result(index,1).los_d.data(i,1) == 0)  % 判断是否存在多径  如果不存在
        m_result(index,1).m(i,1:2) = m_result(index,1).m(i-1,1:2);
        m_result(index,1).P{i}(1:2,1:2) = m_result(index,1).P{i-1}(1:2,1:2);
        m_minus = mpc_result(index,1).m(i-1,1:4)';
        P_m_minus = mpc_result(index,1).P{i-1}(1:4,1:4);  
    else  %如果存在
        xy_minus = m_result(index,1).m(i-1,1:2)';
        P_xy_minus = m_result(index,1).P{i-1}(1:2,1:2) + [Delta_x^2, 0 ; 0 , Delta_y^2];
        % 更新
        vk(1,1) = result(index,1).los_d.data(i,1) - norm(xy_minus);
        vk(2,1) = result(index,1).los_phi.data(i,1) - atan2(xy_minus(2,1), xy_minus(1,1));
        vk(2,1) = wrapToPi(vk(2,1));
        Hxk = Hx_los(xy_minus(1,1),xy_minus(2,1));
        Sk = Hxk*P_xy_minus*Hxk' + [Delta_los_d^2, 0 ;0 , Delta_los_phi^2];
        Kk = P_xy_minus*Hxk'*(Sk^(-1));
        xy_k = xy_minus + Kk*vk;
        P_xy_k = P_xy_minus - Kk*Sk*Kk';

        m_result(index,1).m(i,1:2) = xy_k';
        m_result(index,1).P{i}(1:2,1:2) = P_xy_k;
        %% 拷贝过来 以供多径定位使用
        mpc_result(index,1).m(i,1:2)  = m_result(index,1).m(i,1:2) ;
        mpc_result(index,1).P{i}(1:2,1:2) = m_result(index,1).P{i}(1:2,1:2) ;
        %% 开始进行NLOS_EKF

        m_minus = [mpc_result(index,1).m(i,1:2)';mpc_result(index,1).m(i-1,3:4)'];
        P_m_minus = [mpc_result(index,1).P{i}(1:2,1:2),mpc_result(index,1).P{i-1}(1:2,3:4);mpc_result(index,1).P{i-1}(3:4,1:2),mpc_result(index,1).P{i-1}(3:4,3:4)];
    end
    
    if (result(index,1).mpc_d.data(i,1) == 0)  % 没有多径的时候则不进行更新
        mpc_result(index,1).m(i,1:4) = m_minus';
        mpc_result(index,1).P{i}(1:4,1:4) = P_m_minus;
    else
        [xm,ym] = mirror(m_minus(1,1),m_minus(2,1),m_minus(3,1),m_minus(4,1));
        vk(1,1) = result(index,1).mpc_d.data(i,1) - norm([xm,ym]);
        vk(2,1) = result(index,1).mpc_phi.data(i,1) - atan2(ym,xm);
        vk(2,1) = wrapToPi(vk(2,1));
        Hxk = Hx_mirror(m_minus(1,1),m_minus(2,1),m_minus(3,1),m_minus(4,1));
        Sk = Hxk*P_m_minus*Hxk' + diag([Delta_mpc_d^2, Delta_mpc_phi^2]);
        Kk = P_m_minus*Hxk'*(Sk^(-1));
        m_k = m_minus + Kk * vk;
        P_m_k = P_m_minus - Kk*Sk*Kk';
        mpc_result(index,1).m(i,1:4) = m_k';
        mpc_result(index,1).P{i}(1:4,1:4) = P_m_k;
    end
    
    %% 定位之后可以再使用搜索到的反射面的位置再进行一轮极大似然搜索  
    
%     loc = optimvar('loc',1,2);
%     data = [result(index,1).los_d.data(i,1);result(index,1).los_phi.data(i,1);result(index,1).mpc_d.data(i,1);result(index,1).mpc_phi.data(i,1)];
%     C = diag([Delta_los_d^2,Delta_los_phi^2,Delta_mpc_d^2,Delta_mpc_phi^2]);
%     obj = fcn2optimexpr(@func,loc,mpc_result(index,1).m(i,3),mpc_result(index,1).m(i,4),C,data);
%     prob = optimproblem('Objective',obj);
%     nlcons = loc(1)^2 + loc(2)^2 <= 2;
%     prob.Constraints.cons = nlcons;
%     x0.loc = m_result(index,1).m(i,1:2);
%     [sol,fval] = solve(prob,x0);
%     final_result(index,1).m(i,1:2) = sol.loc;
    
    
end


% error_mat(index,1).atntenna_num = antenna_num;
% for kk = 1:useful_num
%     error_mat(index,1).los_mpc_error(kk,1) = norm(mpc_result(index,1).m(kk,1:2) - target_loc);
%     error_mat(index,1).los_error(kk,1) = norm(m_result(index,1).m(kk,1:2) - target_loc);
% 
% end
% close all;
%  f = figure(antenna_num);
%     hdcdf(1) = cdfplot(error_mat(index,1).los_mpc_error);
%     hold on;
%     hdcdf(2) = cdfplot(error_mat(index,1).los_error);
% 
% 
% 
%     set(hdcdf(1),'color','r','linewidth',1.5)
%     set(hdcdf(2),'color','b','linewidth',1.5)
% 
% 
%     xlabel('Absolute Error [m]');
%     ylabel(' CDF');
%     grid on;
%     set(gca,'FontSize',14);
%     title(" ");
%         axis([0 0.1 0 1]);
% 
%     legend("mpc_result","los_result");
%     
%% 极大似然搜素函数
function p = func(loc,h,theta,C,ob_data)
% loc是实际的位置
% h是反射面的截距
% theta 是反射面的转角
% C 是观测变量的协方差
% ob_data 是观测的变量
%

%% 
     [xm,ym]  = mirror(loc(1),loc(2),h,theta);
     mu = [norm([loc(1),loc(2)]) ;atan2(loc(2),loc(1)) ; norm([xm,ym])  ;atan2(ym,xm)];
%     mu = [norm([loc(1),loc(2)]) ;atan2(loc(2),loc(1)) ; norm([loc(1),loc(2)]) ;atan2(loc(2),loc(1)) ];
    p = -(-1/2 * (ob_data - mu)' * (C^(-1)) * (ob_data - mu));

end













