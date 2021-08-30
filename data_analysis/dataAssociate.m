clear all; clc;close all;
result = loadjson("move_01.json");

%% 数据格式化
antenna_num = 8;
index = antenna_num - 2;
K = 2; %最大多径目标为8个
dataNums = length(result(1,index).mpc_d.data);
sigmaF0 = 0.5;
global sigmaR;
sigmaR = 0.5;
sigmaQ = 0.005;
global Pd;
Pd = 0.9;
iterP =20 + 1;
maxGMMNum = 50;


result(1,index).mpc_phi.data(2,:) = result(1,index).mpc_phi.data(1,:) ;
for n = 1:dataNums
    for m = 1:length(find(result(1,index).mpc_d.data(n,:)))
        Z{n}{m} = [result(1,index).mpc_d.data(n,m), result(1,index).mpc_phi.data(n,m)];
    end
end

%% 设定运动模型
A = eye(2);


%% 初始化模型
for n = 1
    for k = 1:K
        if k<=length(find(result(1,index).mpc_d.data(1,:)))
            F.mu{n}{k}{1} = polar2rect(result(1,index).mpc_d.data(1,k),result(1,index).mpc_phi.data(1,k));
            F.sigma{n}{k}{1} = sigmaF0;
            F.coeff{n}{k}{1} = 1;
        else
            F.mu{n}{k}{1} = F.mu{1}{1}{1};
            F.sigma{n}{k}{1} = sigmaF0;
            F.coeff{n}{k}{1} = 1;
        end
    end
end

%% 进行迭代
for n = 2:dataNums
    n
    %% 预测
    for k = 1:K
        numGMMs = length(F.mu{n-1}{k});
        for g = 1:numGMMs
            Alpha.mu{n}{k}{g} = A * F.mu{n-1}{k}{g};
            Alpha.sigma{n}{k}{g} = sqrt(F.sigma{n-1}{k}{g}^2 + sigmaQ^2);
            Alpha.coeff{n}{k}{g} = F.coeff{n-1}{k}{g};
        end
    end
    %% 量测更新
    MnMat(n,1) = length(find(result(1,index).mpc_d.data(n,:)));
    for k = 1:K
        numGMMs = length(Alpha.coeff{n}{k});
        assoA.beta{n}{k} = [0,1:MnMat(n,1);zeros(1,MnMat(n,1)+1)];
        for a_nk = assoA.beta{n}{k}(1,:)
            a_nk_index = a_nk + 1;
            for g = 1:numGMMs
                G_measure(Alpha.mu{n}{k}{g},a_nk,Z{n});
                assoA.beta{n}{k}(2,a_nk_index) = assoA.beta{n}{k}(2,a_nk_index) +Alpha.coeff{n}{k}{g} * G_measure(Alpha.mu{n}{k}{g},a_nk,Z{n});
            end
        end
    end
    
    %% 数据匹配
    Xi{n}{1} = rand(K,MnMat(n,1));  %随机数初始化
    Nu{n}{1} = rand(MnMat(n,1),K);
    for iter = 2:iterP
        for k = 1:K
            for m = 1:MnMat(n,1)
                
                tempCoeff = 0;
                tempList = 1:MnMat(n,1);
                tempList(find(tempList == m)) = [];
                for m_dot = tempList
                    a_nk_index = find(assoA.beta{n}{k}(1,:) == m_dot);
                    tempCoeff = tempCoeff + assoA.beta{n}{k}(2,a_nk_index) * varphi_ank(m_dot) * Nu{n}{iter -1}(m_dot,k);
                end
                a_nk_index = find(assoA.beta{n}{k}(1,:) == m);
                Xi{n}{iter}(k,m) = (assoA.beta{n}{k}(2,a_nk_index) * varphi_ank(m))/((1-Pd) + tempCoeff);
            end
        end
        
        for m = 1:MnMat(n,1)
            for k = 1:K
                tempCoeff = 0;
                tempList = 1:K;
                tempList(find(tempList == k)) = [];
                for k_dot = tempList
                    tempCoeff = tempCoeff + Xi{n}{iter}(k_dot,m);
                end
                Nu{n}{iter}(m,k) = 1/(1+tempCoeff);
            end
        end
    end
    %% 计算匹配概率
    % 计算a的匹配矩阵
    for k = 1:K
        assoA.P{n}{k} = [0,1:MnMat(n,1);zeros(1,MnMat(n,1)+1)];
        for a_nk = assoA.P{n}{k}(1,:)

            tempCoeff = 0;
            for m_dot = 1:MnMat(n,1)
                a_nk_index = find(assoA.beta{n}{k}(1,:) == m_dot);
                tempCoeff = tempCoeff + assoA.beta{n}{k}(2,a_nk_index) * varphi_ank(m_dot) * Nu{n}{iterP}(m_dot,k);
            end
            tempCoeff = tempCoeff + assoA.beta{n}{k}(2,1) * varphi_ank(0) * 1;
                     
            if a_nk == 0
                a_nk_index = find(assoA.beta{n}{k}(1,:) == a_nk);
                assoA.P{n}{k}(2,a_nk_index) = assoA.beta{n}{k}(2,a_nk_index) * varphi_ank(a_nk) * 1 / tempCoeff;
            else
                a_nk_index = find(assoA.beta{n}{k}(1,:) == a_nk);
                assoA.P{n}{k}(2,a_nk_index) = assoA.beta{n}{k}(2,a_nk_index) * varphi_ank(a_nk) * Nu{n}{iterP}(a_nk,k)/tempCoeff;
            end
        end
    end
    % 计算b匹配矩阵
    for m = 1:MnMat(n,1)
        assoB.P{n}{m} = [0,1:K;zeros(1,K + 1)];
        for b_nm = assoB.P{n}{m}(1,:)
            tempCoeff = 0;
            for k_dot = 1:K
                tempCoeff = tempCoeff + Xi{n}{iterP}(k_dot,m);
            end
            tempCoeff = tempCoeff + 1;
            
            if b_nm == 0
                b_nm_index = find(assoB.P{n}{m}(1,:) == b_nm);
                assoB.P{n}{m}(2,b_nm_index) = 1/tempCoeff;
            else
                b_nm_index = find(assoB.P{n}{m}(1,:) == b_nm);
                assoB.P{n}{m}(2,b_nm_index) = Xi{n}{iterP}(b_nm,m)/tempCoeff;
            end
        end
    end
    
    %% 数据更新
    for k = 1:K
        g = 1;
        for a_nk = assoA.P{n}{k}(1,:)
            if a_nk == 0
            else
                a_nk_index = find(assoA.P{n}{k}(1,:) == a_nk);
                z_nm = Z{n}{a_nk};
                Gamma.mu{n}{k}{g} = polar2rect(z_nm(1),z_nm(2));
                Gamma.sigma{n}{k}{g} = sigmaR;
                Gamma.coeff{n}{k}{g} = assoA.P{n}{k}(2,a_nk_index); 
                g = g + 1;
            end
        end
        
        for g = length(Gamma.coeff{n}{k})
            Gamma.coeff{n}{k}{g} = Gamma.coeff{n}{k}{g} + assoA.P{n}{k}(2,1);
        end
        
    end
    
    for k = 1:K
        numG1 = length(Alpha.coeff{n}{k});
        numG2 = length(Gamma.coeff{n}{k});
        numG3 = numG1 * numG2;
        g3 = 1;
        for g1 = 1:numG1
            for g2 = 1:numG2
                mu1 = Alpha.mu{n}{k}{g1};
                sigma1 = Alpha.sigma{n}{k}{g1};
                coeff1 = Alpha.coeff{n}{k}{g1};
                
                mu2 = Gamma.mu{n}{k}{g2};
                sigma2 = Gamma.sigma{n}{k}{g2};
                coeff2 = Gamma.coeff{n}{k}{g2};
                
                F_back.mu{n}{k}{g3} = (mu1*sigma2^2 + mu2 * sigma1^2)/(sigma1^2 + sigma2^2);
                F_back.sigma{n}{k}{g3} = sqrt((sigma1^2)*(sigma2^2)/(sigma1^2 + sigma2^2));
                F_back.coeff{n}{k}{g3} = coeff1 * coeff2 * 1 / sqrt(2*pi*(sigma1^2 + sigma2^2)) *...
                    exp(-1 * ((norm(mu1 - mu2))^2)/( 2 * (sigma1^2 + sigma2^2))) + 0.0000000000001;
                g3 = g3 + 1;
            end
        end
        
        
%         MaxCoeff = 0;
%         for g3 = 1:numG3
%             if F_back.coeff{n}{k}{g3} > MaxCoeff
%                 MaxCoeff = F_back.coeff{n}{k}{g3};
%                 MaxCoeffIndx = g3;
%             end
%         end
%         
%         F.mu{n}{k}{1} = F_back.mu{n}{k}{MaxCoeffIndx};
%         F.sigma{n}{k}{1} = F_back.sigma{n}{k}{MaxCoeffIndx};
%         F.coeff{n}{k}{1} = 1;

        CoeffMat = cell2mat(F_back.coeff{n}{k});
        [SortCoeff,IndexCoeff] = sort(CoeffMat,'descend');
        if length(SortCoeff)>maxGMMNum
            SortCoeff = SortCoeff(1:maxGMMNum);
            IndexCoeff = IndexCoeff(1:maxGMMNum);
        end
             
        sumCoeff = 0;
        for g3 = 1:length(SortCoeff)
            sumCoeff = sumCoeff + F_back.coeff{n}{k}{IndexCoeff(g3)};
        end
        
        for g3 = 1:length(SortCoeff)
            F.mu{n}{k}{g3} = F_back.mu{n}{k}{IndexCoeff(g3)};
            F.sigma{n}{k}{g3} = F_back.sigma{n}{k}{IndexCoeff(g3)};
            F.coeff{n}{k}{g3} = F_back.coeff{n}{k}{IndexCoeff(g3)}/sumCoeff;
        end
    end
end





