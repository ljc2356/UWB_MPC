function [] = ReadTime(After_records,index,i)
%% 读取本次观测与上一时刻观测时间差
% After_records 过滤后的records 存储基本测距、PDOA、波形信息
% index         index = （N(天线数目） - 2）对N天线测距测角结果Result进行操作
% i             当前时刻索引
%%
    global result;
    if i > 1
        this_time_str = After_records(i,1).uwbResult.rxts(1);
        last_time_str = After_records(i-1 ,1).uwbResult.rxts(1);
        delta_time_str = this_time_str - last_time_str;
        delta_time = double(delta_time_str / (5.8070e+07));  % 这里是以毫秒为单位
        delta_time = delta_time /1000;    %换算成秒为单位
        result(index,1).Delta_time(i,1) = delta_time;
    end
end