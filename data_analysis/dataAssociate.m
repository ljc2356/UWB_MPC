clear all; clc;close all;
result = loadjson("move_01.json");

%% 数据格式化
antenna_num = 8;
index = antenna_num - 2;
K = 8; %最大多径目标为8个
dataNums = length(result(1,index).mpc_d.data);
sigmaF0 = 0.5;
global sigmaR;
sigmaR = 0.1;
iterP =2 + 1;
maxGMMNum = 10;


result(1,index).mpc_phi.data(2,:) = result(1,index).mpc_phi.data(1,:) ;
for n = 1:dataNums
    for m = 1:length(result(1,index).mpc_d.data(1,:))
        Z{n}{m} = [result(1,index).mpc_d.data(n,m), result(1,index).mpc_phi.data(n,m)];
    end
end

%% 设定运动模型
A = eye(2);
sigmaQ = 0.2;

%% 初始化模型
for n = 1
    for k = 1:K
        if k<=length(find(result(1,index).mpc_d.data(1,:)))
            F.mu{n}{k}{1} = polar2rect(result(1,index).mpc_d.data(1,k),result(1,index).mpc_phi.data(1,k));
            F.sigma{n}{k}{1} = sigmaF0;
            F.coeff{n}{k}{1} = 1;
        else
            F.mu{n}{k}{1} = F.mu{1}{2}{1};
            F.sigma{n}{k}{1} = sigmaF0;
            F.coeff{n}{k}{1} = 1;
        end
    end
end

%% 进行迭代
for n = 2:10
    n
    %% 预测
    for k = 1:K
        numGMMs = length(F.mu{n-1}{k});
        for g = 1:numGMMs
            Alpha.mu{n}{k}{g} = A * F.mu{n-1}{k}{g}';
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
        % 数据初始化
    for k = 1:K
        for m = 1:MnMat(n,1)
            Xi{n}{k}{m}{1} = [0,1:K;zeros(1,K+1)]; 
            for b_nm = Xi{n}{k}{m}{1}(1,:)
                b_nm_index = b_nm + 1;
                for a_nk = assoA.beta{n}{k}(1,:)
                    a_nk_index = a_nk + 1;
                    
                    if ((a_nk == m)&&(b_nm ~= k)) || ((b_nm == k)&&(a_nk ~= m))
                        PHI_ankbnm = 0;
                    else
                        PHI_ankbnm = 1;
                    end
     
                    Xi{n}{k}{m}{1}(2,b_nm_index) = Xi{n}{k}{m}{1}(2,b_nm_index) + ...
                        assoA.beta{n}{k}(2,a_nk_index)*varphi_ank(a_nk)*PHI_ankbnm;
                end
            end
        end
    end
        % 开始进行迭代
    for iter = 2:iterP
        for m = 1:MnMat(n,1)
            for k = 1:K
                Nu{n}{m}{k}{iter} = [0,1:MnMat(n,1);zeros(1,MnMat(n,1)+1)];
                for a_nk = Nu{n}{m}{k}{iter}(1,:)
                    a_nk_index = a_nk + 1;
                    for b_nm = Xi{n}{k}{m}{iter -1}(1,:)
                        b_nm_index = b_nm + 1;
                        if ((a_nk == m)&&(b_nm ~= k)) || ((b_nm == k)&&(a_nk ~= m))
                            PHI_ankbnm = 0;
                        else
                            PHI_ankbnm = 1;
                        end
                        
                        tempCoeff = 1;
                        tempList = 1:K;
                        tempList(find(tempList == k)) = [];
                        for k_dot = tempList
                            tempCoeff = tempCoeff * Xi{n}{k_dot}{m}{iter-1}(2,b_nm_index);
                        end
                        tempCoeff = tempCoeff * PHI_ankbnm;
                        Nu{n}{m}{k}{iter}(2,a_nk_index) = Nu{n}{m}{k}{iter}(2,a_nk_index) + tempCoeff;
                    end
                end
            end
        end
        
        for k = 1:K
            for m = 1:MnMat(n,1)
                Xi{n}{k}{m}{iter} = [0,1:K;zeros(1,K+1)]; 
                for b_nk = Xi{n}{k}{m}{iter}(1,:)
                    b_nk_index = b_nk + 1;
                    for a_nk = Nu{n}{m}{k}{iter}(1,:)
                        a_nk_index = a_nk + 1;
                        
                        if ((a_nk == m)&&(b_nm ~= k)) || ((b_nm == k)&&(a_nk ~= m))
                            PHI_ankbnm = 0;
                        else
                            PHI_ankbnm = 1;
                        end
                        
                        tempCoeff = PHI_ankbnm * assoA.beta{n}{k}(2,a_nk_index) * varphi_ank(a_nk);
                        tempList = 1:MnMat(n,1);
                        tempList(find(tempList == m)) = [];
                        for m_dot = tempList
                            tempCoeff = tempCoeff * Nu{n}{m_dot}{k}{iter}(2,a_nk_index);
                        end
                        Xi{n}{k}{m}{iter}(2,b_nk_index) = Xi{n}{k}{m}{iter}(2,b_nk_index) + tempCoeff;
                    end
                end
            end
        end
    end
    
    for k = 1:K
       assoA.eta{n}{k} = [0,1:MnMat(n,1);zeros(1,MnMat(n,1)+1)];
       for a_nk = assoA.eta{n}{k}(1,:)
           a_nk_index = a_nk + 1;
           tempCoeff = varphi_ank(a_nk);
           for m = 1:MnMat(n,1)
               tempCoeff = tempCoeff * Nu{n}{m}{k}{iterP}(2,a_nk_index);
           end
           assoA.eta{n}{k}(2,a_nk_index) = tempCoeff;
       end
    end
    
    %% 数据更新
    for k = 1:K
        for a_nk = assoA.eta{n}{k}(1,:)
            a_nk_index = a_nk + 1;
            g = a_nk;
            if a_nk == 0
            else
                z_nm = Z{n}{a_nk};
                Gamma.mu{n}{k}{g} = polar2rect(z_nm(1),z_nm(2));
                Gamma.sigma{n}{k}{g} = sigmaR;
                Gamma.coeff{n}{k}{g} = assoA.eta{n}{k}(2,a_nk_index); 
            end
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
                
                F.mu{n}{k}{g3} = (mu1*sigma2^2 + mu2 * sigma1^2)/(sigma1^2 + sigma2^2);
                F.sigma{n}{k}{g3} = sqrt((sigma1^2)*(sigma2^2)/(sigma1^2 + sigma2^2));
                F.coeff{n}{k}{g3} = coeff1 * coeff2 * 1 / sqrt(2*pi*(sigma1^2 + sigma2^2)) *...
                    exp(-1 * ((norm(mu1 - mu2))^2)/( 2 * (sigma1^2 + sigma2^2)));
                g3 = g3 + 1;
            end
        end
    end
end



