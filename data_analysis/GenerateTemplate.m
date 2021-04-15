function [ Template, Conv_Template ] = GenerateTemplate(Back_filenames)

records = loadRecordFile(Back_filenames);
[Back_num,Back_records]= UsefulSelect(records);
Signal_length = length(Back_records(1).uwbResult.cir{1,1});
run("Properties.m");

%% 获得模板
Template = zeros(8,Signal_length);
for i = 1:Back_num
    for kk = 1:8

        Template(kk,:) = Template(kk,:) + abs(Back_records(i,1).uwbResult.cir{1,kk}')/max(abs(Back_records(i,1).uwbResult.cir{1,kk}(8,1)));

    end
end
Template = Template/Back_num;
for i = 1:8   %对模板插值
%     Template(i,1:Signal_length*intersize - intersize +1 ) = interp1(1:Signal_length,Template(i,1:Signal_length),1:1/intersize:(Signal_length),'pchip');
    Template(i,1:Signal_length*intersize - intersize +1 ) = sinc_interp(Template(i,1:Signal_length)', intersize, 0)';
end

first_index = center_index - (window_width-1)/2;
second_index = center_index + (window_width-1)/2;
% Conv_Template = Template(:,first:second_index);
Conv_Template = Template(:,  (first_index-1)*(intersize-1) + first_index  : (second_index-1)*(intersize-1) + second_index);   %对模板进行插值
Conv_Template = fliplr(Conv_Template);   %由于后面的相关直接采用卷积的方式，因此对其进行倒向

end