function [useful_num,After_records]= UsefulSelect(records)
%% 将8天线都有数据的情况挑选出来，存储到After_records中
% 挑选出八天线都有数据的情况 
% 但是请注意，这样会在某些时刻增加两次数据之间的时间差
% 输入 
% records 原始记录结构体
% 输出
% useful_num 挑出的记录结构体的长度
% After_records 挑出的记录结构体
%% 
    file_num = length(records);
    useful_num = 0;
    
    for i = 50:file_num %刚开始时一般数据并不好 因此抛弃
        flag = 0;
            for k = 1:8
                RX = abs(records(i).uwbResult.cir{1,k});
                if isempty(RX)
                    break;
                end
                flag = flag +1;
            end
        if (flag == 8 )
            useful_num = useful_num +1;
            After_records(useful_num,1) = records(i);
        end
    end

end