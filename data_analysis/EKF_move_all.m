clear all;
load('data/d_move_01.mat')

global result;
global m_result;

%% 指定参数
% 运动噪声
% result(6,1).Delta_time(find(result(6,1).Delta_time ~= 0.079)) = 0.079;
Delta_u = 0.01;

% 观测噪声
Delta_los_d = 0.126;
Delta_los_phi = 0.056;   % 倒用噪声 倒用后期角度变化大 噪声影响小 因此多径权值比较大
Delta_mpc_d = 0.0601;
Delta_mpc_phi = 0.0080;

% Delta_los_d = 0.126;
% Delta_los_phi = 0.056;   % 倒用噪声 倒用后期角度变化大 噪声影响小 因此多径权值比较大
% Delta_mpc_d = 0.0401;
% Delta_mpc_phi = 0.0040;


% Delta_los_d = 0.0226;
% Delta_los_phi = 0.0046;
% Delta_mpc_d = 0.0226;
% Delta_mpc_phi = 0.0046;  % 这一组噪声是设置是正用的结果 由于正用后期 角度变化小 噪声影响大 因此降低多径权值

%%

antenna_num = 8;
index = antenna_num - 2;
result(index,1).los_d.data(60:170,1) = 0;
% result(index,1).los_d.data(100:230,1) = 0;
m_result(index,1).antenna_num = antenna_num;
m_result(index,1).m(1,:) = [0.55 0 0.1 0 3 0]; % 指定初值
m_result(index,1).P{1} = eye(6)*100;  %相关噪声给大一些
m_result(index,1).Q{1} =  [eye(4)/1000,zeros(4,2);zeros(2,4),zeros(2,2)];   %运动噪声小一点
m_result(index,1).R{1} = [eye(2),zeros(2,2);zeros(2,2),[0.271650346223203,0;0,0.000429199642412428]];     %观测噪声任取
m_result(index,1).e_flat(:,1) = zeros(4,1);
m_result(index,1).w_flat(:,1) = zeros(6,1);

NR = 2;
NQ = 2;
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
        
        m_minus = A_expend * m_result(index,1).m(i-1,1:6)';
        P_m_minus = A_expend *  m_result(index,1).P{i-1}(1:6,1:6) * A_expend' + Qu_expend;
     %% Rk更新
     % 获得观测误差
        [xm,ym] = mirror(m_minus(1,1),m_minus(2,1),m_minus(5,1),m_minus(6,1));


         zk_hat = [norm(m_minus(1:2,1)');
                   atan2(m_minus(2,1), m_minus(1,1));
                   norm([xm,ym]);
                   atan2(ym,xm)];
         zk = [result(index,1).los_d.data(i,1);
                result(index,1).los_phi.data(i,1);
                result(index,1).mpc_d.data(i,1);
                result(index,1).mpc_phi.data(i,1)];
            
         ek = zk- zk_hat;
         ek(2,1) = wrapToPi(ek(2,1));
         ek(4,1) = wrapToPi(ek(4,1));
         Hxk1 = Hx_los(m_minus(1,1),m_minus(2,1));
         Hxk2 = Hx_mirror(m_minus(1,1),m_minus(2,1),m_minus(5,1),m_minus(6,1));
         Hxk = [Hxk1,zeros(2,2);Hxk2];
         
         a2 = (NR - 1)/NR;
         ek_flat = a2* m_result(index,1).e_flat(:,i-1) + (1/NR)*ek;
         Delta_Rk = (1/(NR-1))*(ek - ek_flat)*(ek - ek_flat)' - (1/NR)* Hxk * P_m_minus * Hxk';
         Rk = abs(diag(diag(a2 * m_result(index,1).R{i-1} + Delta_Rk)));
         m_result(index,1).R{i} = Rk;
         m_result(index,1).e_flat(:,i) = ek_flat;
        
        Sk = Hxk*P_m_minus*Hxk' + diag( [ Delta_los_d^2, Delta_los_phi^2,Delta_mpc_d^2,  Delta_mpc_phi^2]);
        Kk = P_m_minus*Hxk'*(Sk^(-1));
        m_k = m_minus + Kk*ek;
        P_m_k = P_m_minus - Kk * Hxk * P_m_minus;
        %% 运动噪声协方差矩阵更新
        a1 = (NQ -1)/NQ;
        wk_hat = m_k - m_minus;
        wk_flat = a1 * m_result(index,1).w_flat(:,i-1) + (1/NQ)*wk_hat;
        Delta_Qk = (1/(NQ-1)) * (wk_hat - wk_flat)*(wk_hat - wk_flat)' + (1/NQ)*(P_m_minus - A_expend * m_result(index,1).P{i-1}(1:6,1:6) * A_expend');
        Qk = abs(diag(diag(a1* m_result(index,1).Q{i-1} + Delta_Qk)));
        m_result(index,1).w_flat(:,i) = wk_flat;
        m_result(index,1).Q{i} = Qk;
        
        m_result(index,1).m(i,1:6) = m_k';
        m_result(index,1).P{i}(1:6,1:6) = P_m_k;
        
end


close all;
M = moviein(useful_num);
a(2,:) = -1:0.001:0;
a(1,:) = 1;
b(1,:) = 1:0.001:2;
b(2,:) = 0;
ab = [a b];
Base(1,1) = 0;
Base(2,1) = 0;
c1(2,:) = -0.2:0.01:0.2;
c1(1,:) = 0.8;

figure(10);
hd(1) = plot(ab(1,:),ab(2,:));
hold on;


hd(3) = scatter(Base(1,:),Base(2,:),60,"sk",'linewidth',2);
hold on;

hd(4) = plot(c1(1,:),c1(2,:));
hold on;


set(hd(1),'color','g','linestyle','-','linewidth',2)
set(hd(4),'color',[1 0.5 0],'linestyle','-','linewidth',2)
xlabel('x');
ylabel('y');
set(gca,'FontSize',12);  
for i = 1:useful_num     % 旋转并记录每个画面

   hd(5) = scatter(m_result(index,1).m(i,1),m_result(index,1).m(i,2),50,"ro");           % 以绘画函数来产生动画
   hold on;

   axis ([-0.5 3 -1.5 1]);
   if i == useful_num
       legend([hd(1),hd(3),hd(4),hd(5)],"Ground Truth","Anchor" ,"Obstacles","LOS Localization",'Location','SouthEast');
   end
   
   M(i) = getframe;          % 抓取画面值
   im=frame2im(M(i));
   [I,map]=rgb2ind(im,256);

    k=i-0;
    if k==1
        imwrite(I,map,"1.gif",'gif','Loopcount',inf,...
            'DelayTime',0.01);%loopcount只是在i==1的时候才有用
    else
        imwrite(I,map,"1.gif",'gif','WriteMode','append',...
            'DelayTime',0.01);%DelayTime用于设置gif文件的播放快慢
    end

end











