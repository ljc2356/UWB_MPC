function [AEKF_Result] = AEKF(AEKF_Result , Time_index , Observation, Contrl, Motion_model, Obser_model,wrap_index, NR,NQ)
% AEKF_Result  滤波结果 （结构体）
%              m 待估计状态参数
%              P 协方差矩阵
%              Q 运动噪声协方差矩阵
%              R 观测噪声协方差矩阵  （除此之外，还有一些跟AEKF相关的参数）
% Time_index   时间序列
% Obser_model  观测模型（结构体）
%              hx 观测函数句柄
%              JHx 观测雅各比矩阵计算函数句柄
% Observation  观测量
% Contrl       控制量
% Motion_model 运动模型（结构体）
%              fx 运动函数句柄
%              JFx 运动雅各比矩阵计算函数句柄
% wrap_index   int 值，待估计参数的第几维度是角度需要做 wrapToPi处理
% NR NQ        AEKF 的相关参数，通常设置为10到200的大小，越大越接近EKF 


    i = Time_index;
    m_last = AEKF_Result.m(i-1 , :)';
    P_last = AEKF_Result.P{i-1};
    state_dim = length(m_last);
    obser_dim = length(Observation);
    
    fx = Motion_model.fx;
    JFx = Motion_model.JFx;
    hx = Obser_model.hx;
    JHx = Obser_model.JHx;
    
    m_minus = fx(m_last,Contrl);
    P_minus = JFx(m_last,Contrl) * P_last * JFx(m_last,Contrl)' + AEKF_Result.Q{i-1};
    
    zk_hat = hx(m_minus);
    Hxk = JHx(m_minus);
    ek = Observation - zk_hat;
    ek(wrap_index) = wrapToPi(ek(wrap_index));
    

    a2 = (NR-1)/NR;
    ek_flat = a2 * AEKF_Result.e_flat(i-1,:)' + 1/NR * ek;
    Delta_Rk = (1/(NR-1))*(ek - ek_flat)*(ek - ek_flat)' - (1/NR)* Hxk * P_minus * Hxk';
    Rk = abs(diag(diag(a2 * AEKF_Result.R{i-1} + Delta_Rk)));
    AEKF_Result.R{i} = Rk;
    AEKF_Result.e_flat(i,:) = ek_flat';    
    
    Sk = Hxk * P_minus * Hxk' + AEKF_Result.R{i};
    Kk = P_minus * Hxk'*(Sk^(-1));
    m_k = m_minus + Kk * ek;
    P_k = P_minus - Kk * Hxk * P_minus;
    AEKF_Result.m(i , :) = m_k';
    AEKF_Result.P{i} = P_k;
    
    a1 = (NQ -1)/NQ;
    wk_hat = m_k - m_minus;
    wk_flat = a1 * AEKF_Result.w_flat(i-1,:)' + 1/NQ *  wk_hat;
    Delta_Qk = 1/(NQ-1) * (wk_hat - wk_flat)*(wk_hat - wk_flat)'+ 1/NQ * ( P_minus - JFx(m_last,Contrl) * P_last * JFx(m_last,Contrl)');
    Qk = abs(diag(diag(a1* AEKF_Result.Q{i-1} + Delta_Qk)));
    AEKF_Result.w_flat(i,:) = wk_flat';
    AEKF_Result.Q{i} = Qk;

end