function [AEKF_Result] = AEKF(AEKF_Result , Time_index , Observation, Contrl, Motion_model, Obser_model,wrap_index, NR,NQ)
%% AEKF 自适应卡尔曼滤波核心算法框架
% AEKF_Result  滤波信息存储结构体，存储了历史上所有的状态m、状态估计协方差矩阵P、观测噪声协方差矩阵R、运动噪声协方差矩阵Q，运动噪声残差w_flat,观测噪声残差e_flat
% Time_index   当前时刻索引，即当前需要需要被滤波的时间序列索引
% Observation  当前时刻索引所对应的观测
% Contrl       当前时刻索引控制量，与运动方程油管
% Motion_model 运动方程函数句柄结构体，包括fx运动方程句柄，JFx运动方程雅各比矩阵句柄
% Obser_model  观测方程函数句柄结构体，包括hx观测方程句柄，JFx观测方程雅各比矩阵句柄
% wrap_index   观测残差向量中，需要归一化到-pi 到 pi 的值的位置
% NR           AEKF 观测噪声更新相关常数， NR越大越接近EKF
% NQ           AEKF 运动噪声更新相关常数， NQ越大越接近EKF
%%
%获得上一时刻的估计值和协方差
    i = Time_index;
    m_last = AEKF_Result.m(i-1 , :)';   
    P_last = AEKF_Result.P{i-1};
    state_dim = length(m_last);       
    obser_dim = length(Observation);
%初始化运动、观测方程函数    
    fx = Motion_model.fx;  
    JFx = Motion_model.JFx;   
    hx = Obser_model.hx;
    JHx = Obser_model.JHx;
    
%时间更新
    m_minus = fx(m_last,Contrl);    
    P_minus = JFx(m_last,Contrl) * P_last * JFx(m_last,Contrl)' + AEKF_Result.Q{i-1};
    
%预期观测值   
    zk_hat = hx(m_minus);  
    Hxk = JHx(m_minus);
    ek = Observation - zk_hat;  %观测残差
    ek(wrap_index) = wrapToPi(ek(wrap_index));  %对残差中角度数据归一化到 wrap到-Pi 到 pi之间
    
%自适应更新观测噪声协方差
    a2 = (NR-1)/NR;   
    ek_flat = a2 * AEKF_Result.e_flat(i-1,:)' + 1/NR * ek;
    Delta_Rk = (1/(NR-1))*(ek - ek_flat)*(ek - ek_flat)' - (1/NR)* Hxk * P_minus * Hxk';
    Rk = abs(diag(diag(a2 * AEKF_Result.R{i-1} + Delta_Rk)));
    AEKF_Result.R{i} = Rk;
    AEKF_Result.e_flat(i,:) = ek_flat';    
    
%量测更新    
    Sk = Hxk * P_minus * Hxk' + AEKF_Result.R{i}; 
    Kk = P_minus * Hxk'*(Sk^(-1));
    m_k = m_minus + Kk * ek;    
    P_k = P_minus - Kk * Hxk * P_minus;
    AEKF_Result.m(i , :) = m_k';            
    AEKF_Result.P{i} = P_k;
    
%更新运动噪声协方差    
    a1 = (NQ -1)/NQ;
    wk_hat = m_k - m_minus;   
    wk_flat = a1 * AEKF_Result.w_flat(i-1,:)' + 1/NQ *  wk_hat;
    Delta_Qk = 1/(NQ-1) * (wk_hat - wk_flat)*(wk_hat - wk_flat)'+ 1/NQ * ( P_minus - JFx(m_last,Contrl) * P_last * JFx(m_last,Contrl)');
    Qk = abs(diag(diag(a1* AEKF_Result.Q{i-1} + Delta_Qk)));
    AEKF_Result.w_flat(i,:) = wk_flat';
    AEKF_Result.Q{i} = Qk;      

end