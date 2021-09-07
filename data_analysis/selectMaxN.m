function GMMResult = selectMaxN(GMMData,maxN)
%% 对权值进行排序
    coeffMat = reshape(GMMData.coeff,1,[]);
    [sortCoeff,indexCoeff] = sort(coeffMat,'descend');
    if length(indexCoeff)>maxN
        sortCoeff = sortCoeff(1:maxN);
        indexCoeff = indexCoeff(1:maxN);
    end
%% 计算权值之和 归一化因子
    sumCoeff = sum(sortCoeff);
    
%% 得到归一化，缩减长度之后的GMM模型
    for g3 = 1:length(sortCoeff)
        GMMResult.mu(g3,1:2) = GMMData.mu(indexCoeff(g3),1:2);
        GMMResult.sigma(g3,1) = GMMData.sigma(indexCoeff(g3),1);
        GMMResult.coeff(g3,1) = GMMData.coeff(indexCoeff(g3),1) / sumCoeff;
    end
    
end