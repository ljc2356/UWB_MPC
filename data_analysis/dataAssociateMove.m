clear all; clc; close all;
result_ori = loadjson("move_01.json");

iterResult = 200;
UsefulIndex =427:430;

mpc_d_bak = result_ori(1,6).mpc_d.data(UsefulIndex,:);
mpc_d_bak = [mpc_d_bak;mpc_d_bak];
mpc_phi_bak = result_ori(1,6).mpc_phi.data(UsefulIndex,:);
mpc_phi_bak =[mpc_phi_bak;mpc_phi_bak];

% mpc_d_bak(5,1) = mpc_d_bak(5,1) + 2;
% mpc_d_bak(6,1) = mpc_d_bak(6,1) + 2;
% mpc_d_bak(7,1) = mpc_d_bak(7,1) + 2;
% mpc_d_bak(8,1) = mpc_d_bak(8,1) + 2;

result(1,6).mpc_d.data = [];
result(1,6).mpc_phi.data = [];
for i = 1:iterResult
    result(1,6).mpc_d.data = [result(1,6).mpc_d.data;mpc_d_bak];
    result(1,6).mpc_phi.data = [result(1,6).mpc_phi.data;mpc_phi_bak];
end

tempList = [1:500,1100:1600];
for i = tempList
    result(1,6).mpc_d.data(i,1) = result(1,6).mpc_d.data(i,2);
    result(1,6).mpc_phi.data(i,1) = result(1,6).mpc_phi.data(i,2);
    result(1,6).mpc_d.data(i,2:end) = 0;
    result(1,6).mpc_phi.data(i,2:end) = 0;
end


%% 数据格式化
antenna_num = 8;
index = antenna_num - 2;
dataNums = length(result(1,index).mpc_d.data);
Ps = 0.3;    %旧目标在新时刻仍然出现的概率
global Pd;    %如果目标出现，被探测到的概率
Pd = 0.8;
iterP = 15 + 1; %数据匹配迭代轮数
maxGMMNum = 15; %混合高斯模型
sigmaX0 = 0.1;  %初始位置的方差
sigmaQ = 0.01;  %运动噪声
global sigmaR;  %观测噪声
sigmaR = 0.2;
global mu_C ;       %虚警数目服从泊松分布，设置泊松分布均值
mu_C = 1;
global mu_N;    %新目标数目服从泊松分布，设置均值
mu_N = 1;
sigmaBig = 10000;
Pth_legacy = 0.5;     %旧点剪枝阈值
Pth_new = 0.3;        %新点剪枝阈值


% result(1,index).mpc_phi.data(2,:) = result(1,index).mpc_phi.data(1,:) ;
for n = 1:dataNums
    for m = 1:length(find(result(1,index).mpc_d.data(n,:)))
        Z{n}{m} = [result(1,index).mpc_d.data(n,m), result(1,index).mpc_phi.data(n,m)];
    end
end
%% 设定运动模型
A = eye(2);
%% 初始化模型
for k = 1
    for j = 1
        f_tilde{k}{j}.x_r1.mu(1,1:2) = [reshape(polar2rect(Z{k}{j}(1),Z{k}{j}(2)),1,[])];
        f_tilde{k}{j}.x_r1.sigma(1,1) = sigmaX0;
        f_tilde{k}{j}.x_r1.coeff(1,1) = 1;
        
        f_tilde{k}{j}.x_r0.coeff(1,1) = 0;
    end
end

%% 进行迭代定位
for k = 2:dataNums
    k
    %% 预测
    j_k_minus = length(f_tilde{k-1});
    for j = 1:j_k_minus
        numGMMs = length(f_tilde{k-1}{j}.x_r1.coeff(:,1));
        for g = 1:numGMMs
            Alpha{k}{j}.x_r1.mu(g,1:2) = (A * (f_tilde{k-1}{j}.x_r1.mu(g,1:2))')';
            Alpha{k}{j}.x_r1.sigma(g,1) = sqrt(f_tilde{k-1}{j}.x_r1.sigma(g,1)^2 + sigmaQ^2);
            Alpha{k}{j}.x_r1.coeff(g,1) = f_tilde{k-1}{j}.x_r1.coeff(g,1);
        end
        Alpha{k}{j}.x_r0.coeff(1,1) = f_tilde{k-1}{j}.x_r0.coeff(1,1) + (1 - Ps) * (1 - f_tilde{k-1}{j}.x_r0.coeff(1,1));
    end
    
    %% 测量评估
    m_k = length(Z{k});
    Beta{k} = zeros(j_k_minus,m_k+1);
    Eta{k} = zeros(m_k , j_k_minus + 1);
    for j = 1:j_k_minus
        for a_kj = 0:m_k
            j_index = j;
            a_kj_index = a_kj + 1;
            numGMMs = length(Alpha{k}{j}.x_r1.coeff);
            for g = 1:numGMMs
                Beta{k}(j,a_kj_index) = Beta{k}(j,a_kj_index) + Alpha{k}{j}.x_r1.coeff(g,1) * Q_measure(Alpha{k}{j}.x_r1.mu(g,:),1,a_kj,Z{k});
            end
            
            Beta{k}(j,a_kj_index) = Beta{k}(j,a_kj_index) + oneFunction(a_kj) * Alpha{k}{j}.x_r0.coeff(1,1);
        end
    end
    
    for m = 1:m_k
        for b_km = 0:j_k_minus
            m_index = m;
            b_km_index = b_km + 1;
            Z_km = Z{k}{m};
            XY_measure = polar2rect(Z_km(1),Z_km(2));
            Eta{k}(m_index,b_km_index) = mu_N/mu_C * normpdf(XY_measure(1),0,sigmaR) * normpdf(XY_measure(2),0,sigmaR) + 1;
        end
    end
    
    %% 迭代
    Nu{k}{1} = rand(m_k,j_k_minus);    
    Varphi{k}{1} = rand(j_k_minus,m_k);
    for iter = 2:iterP
        for m = 1:m_k
            for j = 1:j_k_minus
                j_index = j + 1;
                tempList = 1:j_k_minus;
                tempList(find(tempList == j)) = [];
                tempCoeff = 0;
                for j_dot = tempList
                    j_dot_index = j_dot + 1;
                    tempCoeff = tempCoeff + Eta{k}(m,j_dot_index) * Varphi{k}{iter -1}(j_dot,m);
                end
                Nu{k}{iter}(m,j) = Eta{k}(m,j_index) / (Eta{k}(m,1) + tempCoeff);
            end
        end
        
        for j = 1:j_k_minus
            for m = 1:m_k
                m_index = m + 1;
                tempList = 1:m_k;
                tempList(find(tempList == m)) = [];
                tempCoeff = 0;
                for m_dot = tempList
                    m_dot_index = m_dot + 1;
                    tempCoeff = tempCoeff + Beta{k}(j,m_dot_index) * Nu{k}{iter}(m_dot,j);
                end
                Varphi{k}{iter}(j,m) = Beta{k}(j,m_index)/(Beta{k}(j,1) + tempCoeff);
            end
        end
    end
    
    for j = 1:j_k_minus
        for a_kj = 0:m_k
            a_kj_index = a_kj + 1;
            if a_kj == 0
                Kappa{k}(j,a_kj_index) = 1;
            else
                m = a_kj;
                Kappa{k}(j,a_kj_index) = Nu{k}{iterP}(m,j);
            end
        end
    end
    
    for m = 1:m_k
        for b_km = 0:j_k_minus
            b_km_index = b_km + 1;
            if b_km == 0
                Iota{k}(m,b_km_index) = 1;
            else
                j = b_km;
                Iota{k}(m,b_km_index) = Varphi{k}{iter}(j,m);
            end
        end
    end
    %% 量测更新
    for j = 1:j_k_minus
        for a_kj = 0:m_k
            a_kj_index = a_kj + 1;
            g = a_kj + 1;
            if a_kj == 0
                Gamma{k}{j}.x_r1.mu(g,1:2) = [0,0];
                Gamma{k}{j}.x_r1.sigma(g,1) = sigmaBig;
                Gamma{k}{j}.x_r1.coeff(g,1) = (1 - Pd) * 1 * Kappa{k}(j,1);
            else
                Z_km = Z{k}{a_kj};
                XY_measure = polar2rect(Z_km(1),Z_km(2));
                Gamma{k}{j}.x_r1.mu(g,1:2) = reshape(XY_measure,1,[]);
                Gamma{k}{j}.x_r1.sigma(g,1) = sigmaR;
                Gamma{k}{j}.x_r1.coeff(g,1) = Pd / mu_C * Kappa{k}(j,a_kj_index);
            end
        end
        Gamma{k}{j}.x_r0.coeff(1,1) = Kappa{k}(j,1);
    end
    
    for m = 1:m_k
        Z_km = Z{k}{m};
        XY_measure = polar2rect(Z_km(1),Z_km(2));
        Varsigma{k}{m}.x_r1.mu(1,1:2) = reshape(XY_measure,1,[]);
        Varsigma{k}{m}.x_r1.sigma(1,1) = sigmaR;
        Varsigma{k}{m}.x_r1.coeff(1,1) = mu_N/mu_C * Iota{k}(m,1);
        
        Varsigma{k}{m}.x_r0.coeff(1,1) = sum(Iota{k}(m,:));
    end
    %% 概率计算
    j_k = j_k_minus + m_k;
    for j = 1:j_k_minus
        numG1 = length(Alpha{k}{j}.x_r1.coeff(:,1));
        numG2 = length(Gamma{k}{j}.x_r1.coeff(:,1));
        numG3 = numG1 * numG2;
        g3 = 1;
        for g1 = 1:numG1
            for g2 = 1:numG2
                mu1 = Alpha{k}{j}.x_r1.mu(g1,1:2);
                sigma1 = Alpha{k}{j}.x_r1.sigma(g1,1);
                coeff1 = Alpha{k}{j}.x_r1.coeff(g1,1);
                
                mu2 = Gamma{k}{j}.x_r1.mu(g2,1:2);
                sigma2 = Gamma{k}{j}.x_r1.sigma(g2,1);
                coeff2 = Gamma{k}{j}.x_r1.coeff(g2,1);
                
                [mu3,sigma3,coeff3] = multiGM(mu1,sigma1,coeff1,mu2,sigma2,coeff2);
                
                f_tilde{k}{j}.x_r1.mu(g3,1:2) = mu3;
                f_tilde{k}{j}.x_r1.sigma(g3,1) = sigma3;
                f_tilde{k}{j}.x_r1.coeff(g3,1) = coeff3;
                
                g3 = g3 + 1;
            end
        end
        
        f_tilde{k}{j}.x_r1 = selectMaxN(f_tilde{k}{j}.x_r1,maxGMMNum);  %限制混合高斯分布数目
        C_kj = sum(f_tilde{k}{j}.x_r1.coeff) + Alpha{k}{j}.x_r0.coeff * Gamma{k}{j}.x_r0.coeff;  % 计算归一化因子
        f_tilde{k}{j}.x_r1.coeff = f_tilde{k}{j}.x_r1.coeff/C_kj;  %归一化 r = 1
        f_tilde{k}{j}.x_r0.coeff(1,1) = Alpha{k}{j}.x_r0.coeff * Gamma{k}{j}.x_r0.coeff / C_kj;  %归一化 r=0
    end
    
    for m = 1:m_k
        j = m + j_k_minus;
        
        C_km = sum(Varsigma{k}{m}.x_r1.coeff) + Varsigma{k}{m}.x_r0.coeff;
        f_tilde{k}{j}.x_r1.mu(1,1:2) = Varsigma{k}{m}.x_r1.mu(1,1:2);
        f_tilde{k}{j}.x_r1.sigma(1,1) = Varsigma{k}{m}.x_r1.sigma(1,1);
        f_tilde{k}{j}.x_r1.coeff(1,1) = Varsigma{k}{m}.x_r1.coeff(1,1)/C_km;
        
        f_tilde{k}{j}.x_r0.coeff(1,1) = Varsigma{k}{m}.x_r0.coeff / C_km;
    end
    %% 剪枝
    tempList = [];
    for j = 1:j_k
        
        if j <= j_k_minus
            Pth = Pth_legacy;
        else
            Pth = Pth_new;
        end
        
        
        Pr_tilde{k}{j}(1) = f_tilde{k}{j}.x_r0.coeff;
        Pr_tilde{k}{j}(2) = sum(f_tilde{k}{j}.x_r1.coeff);
        if Pr_tilde{k}{j}(2)>= Pth
            f_tilde{k}{j}.x_r1.coeff = f_tilde{k}{j}.x_r1.coeff / Pr_tilde{k}{j}(2);
            f_tilde{k}{j}.x_r0.coeff(1,1) = 0;
        else
            tempList = [tempList,j];
        end
    end
    f_tilde{k}(tempList) = [];

end
    

