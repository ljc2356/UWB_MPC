useful_num = length(After_records);
for i = 1:useful_num
    for k = 1:8
        
        los_phase(1,k) = angle(After_records(i,1).uwbResult.cir{1,k}(7 ,1));
        los_pdoa(1,k) = wrapToPi(los_phase(1,k) - los_phase(1,1))  ;
    end
        mycali(i,:) = wrapToPi(los_pdoa - After_records(i,1).meaResult.pdoa);
        
end


%从这个实验可以看出 所有的PDOA都是相位减去1天线相位