%% 参数配置列表 在各个子函数之间复用 

%% 校准数据（如果要修改，需要与校准json联合修改）
cali = wrapToPi([0,-3.8786237648942614,-0.1800383535470497,-3.8974087631917986,-2.3507412761762176,2.084888347044676,6.7786643525842818,0.17383465169886048]);

%% PDOA测角 天线校准参数
fc = 3.9936e+9;
c = 299792458;
radius = 0.0732;


formation{3} = [1 2 3];
formation{4} = [1 2 3 4];
formation{5} = [1 2 3 4 5];
formation{6} = [1 2 3 4 5 6];
formation{7} = [1 2 3 4 5 6 7 ];
formation{8} = [1 2 3 4 5 6 7 8];

%% 多径提取参数

los_path = 8;  %首达径默认为8

window_width =9;   % 注意 这里要求窗口得长度是一个奇数
first_start_window = 10 - (window_width-1)/2;
center_index = 10;

intersize = 64;
MinPeakProminence = 1;

%% 场景参数 这里假设是垂直场景
mirror_distance = -4;
target_loc = [1,0];
%% 角度估计卡尔曼滤波参数
Sigma_angle_move = 0.05;
Sigma_angle_ob = 0.05;






