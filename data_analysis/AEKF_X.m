function [AEKF_Result] = AEKF_X(AEKF_Result, Time_index, Observation, Contrl, Motion_model, Obser_model,wrap_index, NR,NQ,TD)
%% AEKF adaptive fault detection Kalman filter core algorithm framework
% AEKF_Result
% Filtering information storage structure, which stores all the historical state m, state estimation covariance matrix P, observation noise covariance matrix R, motion noise covariance matrix Q, motion noise residual w_flat, observation noise residual e_flat, fault Indicator vector error_index
% Time_index The current time index, that is, the current time series index that needs to be filtered
% Observation Observation corresponding to the current time index
% Contrl Index control quantity at the current moment, and the motion equation tubing
% Motion_model motion equation function handle structure, including fx motion equation handle, JFx motion equation Jacobian matrix handle
% Obser_model Observation equation function handle structure, including hx observation equation handle, JFx observation equation Jacobian matrix handle
% wrap_index In the observed residual vector, it needs to be normalized to the position of the value from -pi to pi
% NR AEKF Observation noise updates the correlation constant, the larger the NR, the closer to the EKF
% NQ AEKF Update the correlation constant of motion noise, the larger the NQ, the closer to EKF
% TD failure threshold, the larger the TD, the stronger the tolerance for errors
%%
% Get the estimated value and covariance of the previous moment
    i = Time_index;
    m_last = AEKF_Result.m(i-1, :)';
    P_last = AEKF_Result.P{i-1};
    state_dim = length(m_last);
    obser_dim = length(Observation);
    
%Initialize motion, observation equation function
    fx = Motion_model.fx;
    JFx = Motion_model.JFx;
    hx = Obser_model.hx;
    JHx = Obser_model.JHx;
% Time update
    m_minus = fx(m_last,Contrl);
    P_minus = JFx(m_last,Contrl) * P_last * JFx(m_last,Contrl)' + AEKF_Result.Q{i-1};
% Observed value prediction
    zk_hat = hx(m_minus);
    Hxk = JHx(m_minus);
    ek = Observation-zk_hat;% prediction observation and actual observation residual
    ek(wrap_index) = wrapToPi(ek(wrap_index));% normalize the angle data in the residual to wrap to -Pi to pi
% Observed noise covariance update
    a2 = (NR-1)/NR;
    ek_flat = a2 * AEKF_Result.e_flat(i-1,:)' + 1/NR * ek;
    Delta_Rk = (1/(NR-1))*(ek-ek_flat)*(ek-ek_flat)'-(1/NR)* Hxk * P_minus * Hxk';
    Rk = abs(diag(diag(a2 * AEKF_Result.R{i-1} + Delta_Rk)));
    
    Sk = Hxk * P_minus * Hxk' + Rk;
% Start to check whether it is the correct estimate
    lambda_k = ek' * Sk^(-1) * ek;
    if (lambda_k<= TD)% If the observed value and estimated value are detected to be correct, output the current calculation result
        AEKF_Result.R{i} = Rk;
        AEKF_Result.e_flat(i,:) = ek_flat';

        Kk = P_minus * Hxk'*(Sk^(-1));
        m_k = m_minus + Kk * ek;
        P_k = P_minus-Kk * Hxk * P_minus;
        AEKF_Result.m(i, :) = m_k';
        AEKF_Result.P{i} = P_k;

        a1 = (NQ -1)/NQ;
        wk_hat = m_k-m_minus;
        wk_flat = a1 * AEKF_Result.w_flat(i-1,:)' + 1/NQ * wk_hat;
        Delta_Qk = 1/(NQ-1) * (wk_hat-wk_flat)*(wk_hat-wk_flat)' + 1/NQ * (P_minus-JFx(m_last,Contrl) * P_last * JFx(m_last,Contrl)');
        Qk = abs(diag(diag(a1* AEKF_Result.Q{i-1} + Delta_Qk)));
        AEKF_Result.w_flat(i,:) = wk_flat';
        AEKF_Result.Q{i} = Qk;
        AEKF_Result.error_index(i,1) = 0;
    else% If an error is detected in the observation, the state and the state covariance adopt the predicted value, and the noise model will continue to use the previous time value
        AEKF_Result.R{i} = AEKF_Result.R{i-1};
        AEKF_Result.e_flat(i,:) = AEKF_Result.e_flat(i-1,:);
        AEKF_Result.m(i, :) = m_minus';
        AEKF_Result.P{i} = P_minus;
        AEKF_Result.w_flat(i,:) = AEKF_Result.w_flat(i-1,:);
        AEKF_Result.Q{i} = AEKF_Result.Q{i-1};
        AEKF_Result.error_index(i,1) = 1;
    end

end