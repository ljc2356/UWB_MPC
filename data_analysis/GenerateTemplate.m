function [ Template, Conv_Template ] = GenerateTemplate(Back_filenames)
%% 生成匹配滤波模板算法
%输入
%Back_filenames 存放背景波形数据文件路径
%输出
%Template       根据背景探测波形，取平均，并插值取得的模板模型
%Conv_Template  从模板波形中取得的有信号部分（用以进行匹配滤波）
%% 读取背景波形
records = loadRecordFile(Back_filenames);
[Back_num,Back_records]= UsefulSelect(records);
Signal_length = length(Back_records(1).uwbResult.cir{1,1});
run("Properties.m");   %读取配置文件

%% 获得模板以及信号模板
Template = zeros(8,Signal_length);
for i = 1:Back_num
    for kk = 1:8
        % 模板采用波形绝对值
        Template(kk,:) = Template(kk,:) + abs(Back_records(i,1).uwbResult.cir{1,kk}')/max(abs(Back_records(i,1).uwbResult.cir{1,kk}(8,1)));
    end
end
Template = Template/Back_num;

%对模板插值
for i = 1:8   
%     Template(i,1:Signal_length*intersize - intersize +1 ) = interp1(1:Signal_length,Template(i,1:Signal_length),1:1/intersize:(Signal_length),'pchip');
    Template(i,1:Signal_length*intersize - intersize +1 ) = sinc_interp(Template(i,1:Signal_length)', intersize, 0)';
end

%提取信号部分
first_index = center_index - (window_width-1)/2;
second_index = center_index + (window_width-1)/2;
% Conv_Template = Template(:,first:second_index);
Conv_Template = Template(:,  (first_index-1)*(intersize-1) + first_index  : (second_index-1)*(intersize-1) + second_index);   %对模板进行插值
Conv_Template = fliplr(Conv_Template);   %由于后面的相关匹配滤波直接采用卷积的方式，因此对其进行倒置 
end