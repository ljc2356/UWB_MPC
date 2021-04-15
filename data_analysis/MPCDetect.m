function [mpc_index] = MPCDetect(Signal_Cir,Template,Conv_Template)
% Signal 每一条输入的数据
% Template 8天线的降噪模板
% Conv_Template 8天线的匹配模板
    run("Properties.m");
    Signal_length = length(Signal_Cir{1,1});
    for kk = 1:8
        This_Sig_tem = abs(Signal_Cir{1,kk}')/max(abs(Signal_Cir{1,kk}(8,1))); %这里就是针对第八位置做归一化

%       This_Sig = interp1(1:Signal_length,This_Sig_tem(1:Signal_length),1:1/intersize:(Signal_length),'pchip');
        This_Sig = sinc_interp(This_Sig_tem(1:Signal_length)', intersize, 0)';
        Filter_Sig(kk,:) = (This_Sig - Template(kk,:));
        Conv_Sig_temp(kk,:) = conv(Filter_Sig(kk,:),Conv_Template(kk,:));

    end
    

    mean_Conv_sig(1,:) = mean(Conv_Sig_temp,1);
    mean_Conv_sig(2,:) = 1:1/intersize:(Signal_length+window_width-1);
    [~,locs] = findpeaks(mean_Conv_sig(1,:),'MinPeakProminence',MinPeakProminence); 
    values = mean_Conv_sig(1,locs); % 峰值的大小
    locs_mat = mean_Conv_sig(2,locs);  % 峰值的位置
    locs_real = (locs_mat -6);  % 根据实际峰值校准得到，与模板的宽度和中心有关  并取整   

    real_index1 = find( (locs_mat > (18 + 6)));
    real_index2 = find( locs_mat < (50+6));
    real_index = intersect(real_index1,real_index2);
    if (isempty(real_index) == 0)
    max_index = find(values == max(values(real_index)));
    this_locs = locs_mat(max_index) - 7;
        %mpc_index(1:length(locs)) = locs;
    mpc_index(1,:) = (this_locs);   

    else
        mpc_index(1) = 0;
    end
end