function [AEKF_Result]  = AEKF_X(AEKF_Result , Time_index , Observation, Contrl, Motion_model, Obser_model,wrap_index, NR,NQ,TD)
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

    
    Sk = Hxk * P_minus * Hxk' + Rk;
    % 开始检验是否是正确的估计值
    lambda_k = ek' * Sk^(-1) * ek;
    if (lambda_k<= TD)
        AEKF_Result.R{i} = Rk;
        AEKF_Result.e_flat(i,:) = ek_flat';    

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
        AEKF_Result.error_index(i,1) = 0;
    else
        AEKF_Result.R{i} = AEKF_Result.R{i-1};
        AEKF_Result.e_flat(i,:) = AEKF_Result.e_flat(i-1,:);
        AEKF_Result.m(i , :) = m_minus';
        AEKF_Result.P{i} = P_minus;
        AEKF_Result.w_flat(i,:)= AEKF_Result.w_flat(i-1,:);
        AEKF_Result.Q{i} =  AEKF_Result.Q{i-1};
        AEKF_Result.error_index(i,1) = 1;
    end

end