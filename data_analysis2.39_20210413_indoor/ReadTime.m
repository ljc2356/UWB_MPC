function [] = ReadTime(After_records,index,i)
    global result;
%     this_time = str2double(After_records(i,1).time(end-2:end));   %随后应当使用时间戳来
%     last_time
    if i > 1
        this_time_str = After_records(i,1).uwbResult.rxts(1);
        last_time_str = After_records(i-1 ,1).uwbResult.rxts(1);
        delta_time_str = this_time_str - last_time_str;
        delta_time = double(delta_time_str / (5.8070e+07));  % 这里是以毫秒为单位
        delta_time = delta_time /1000;    %换算成秒为单位
        result(index,1).Delta_time(i,1) = delta_time;
    end
end