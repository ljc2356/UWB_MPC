function [AEKF_Result] = AEKF(AEKF_Result , Time_index , Observation, Contrl, Motion_model, Obser_model,wrap_index, NR,NQ)
%% AEKF Core algorithm framework of adaptive Kalman filter
% AEKF_Result  The filtering information storage structure has stored all the historical state M, state estimation covariance matrix P, observation noise covariance matrix R, motion noise covariance matrix Q, motion noise residual W_flat, observation noise residual E_flat
% Time_index   The index of the current moment, that is, the index of the current time series that needs to be filtered
% Observation  The observation corresponding to the current time index

% Contrl       The controlling quantity at the current time is related to the equation of motion
% Motion_model Equation of motion function handle structure, including fx equation of motion handle, JFx equation of motion Jacoby matrix handle
% Obser_model  Observational equation function handle structure, including HX observation equation handle, JFX observation equation Jacoby matrix handle
% wrap_index   In the observation residual vector, it needs to be normalized to the position of the value from -pi to PI
% NR           AEKF observation noise updates the correlation constant, and the larger NR is, the closer it is to EKF
% NQ           The moving noise of AEKF updates the correlation constant, and the larger NQ is, the closer it is to EKF
%%
%Obtain the estimate and covariance of the previous time
    i = Time_index;
    m_last = AEKF_Result.m(i-1 , :)';   
    P_last = AEKF_Result.P{i-1};
    state_dim = length(m_last);       
    obser_dim = length(Observation);
%Initialize the motion and observe the equation function  
    fx = Motion_model.fx;  
    JFx = Motion_model.JFx;   
    hx = Obser_model.hx;
    JHx = Obser_model.JHx;
    
%Predict
    m_minus = fx(m_last,Contrl);    
    P_minus = JFx(m_last,Contrl) * P_last * JFx(m_last,Contrl)' + AEKF_Result.Q{i-1};
    
%Expected observed value 
    zk_hat = hx(m_minus);  
    Hxk = JHx(m_minus);
    ek = Observation - zk_hat;  %The observation residuals
    ek(wrap_index) = wrapToPi(ek(wrap_index));  %The Angle data in the residual is normalized to wrap to -pi to Pi
    
%Adaptively update the observation noise covariance
    a2 = (NR-1)/NR;   
    ek_flat = a2 * AEKF_Result.e_flat(i-1,:)' + 1/NR * ek;
    Delta_Rk = (1/(NR-1))*(ek - ek_flat)*(ek - ek_flat)' - (1/NR)* Hxk * P_minus * Hxk';
    Rk = abs(diag(diag(a2 * AEKF_Result.R{i-1} + Delta_Rk)));
    AEKF_Result.R{i} = Rk;
    AEKF_Result.e_flat(i,:) = ek_flat';    
    
%Measurement update 
    Sk = Hxk * P_minus * Hxk' + AEKF_Result.R{i}; 
    Kk = P_minus * Hxk'*(Sk^(-1));
    m_k = m_minus + Kk * ek;    
    P_k = P_minus - Kk * Hxk * P_minus;
    AEKF_Result.m(i , :) = m_k';            
    AEKF_Result.P{i} = P_k;
    
%Update the motion noise covariance
    a1 = (NQ -1)/NQ;
    wk_hat = m_k - m_minus;   
    wk_flat = a1 * AEKF_Result.w_flat(i-1,:)' + 1/NQ *  wk_hat;
    Delta_Qk = 1/(NQ-1) * (wk_hat - wk_flat)*(wk_hat - wk_flat)'+ 1/NQ * ( P_minus - JFx(m_last,Contrl) * P_last * JFx(m_last,Contrl)');
    Qk = abs(diag(diag(a1* AEKF_Result.Q{i-1} + Delta_Qk)));
    AEKF_Result.w_flat(i,:) = wk_flat';
    AEKF_Result.Q{i} = Qk;      

end