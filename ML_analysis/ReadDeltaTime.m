function [delta_time] = ReadDeltaTime(After_records,i)

    if i > 1
        this_time_str = After_records(i,1).uwbResult.rxts(1);
        last_time_str = After_records(i-1 ,1).uwbResult.rxts(1);
        delta_time_str = this_time_str - last_time_str;
        delta_time = double(delta_time_str / (5.8070e+07));  % 这里是以毫秒为单位
        delta_time = delta_time /1000;    %换算成秒为单位
    else
        delta_time = 0;
    end
end