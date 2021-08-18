%% 首次校准数据（如果要修改，需要与校准json联合修改）
cali = wrapToPi([0,-3.8982133612189145,-0.099606486317065812,-3.720636379939358,-2.3037893425940075,2.0705191871578319,6.7912329961149389,0.064182012127846383]);


%% 二次校准参数
los_d_truth = 1;
los_phi_truth = 0;  % 设定groundtruth;

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
MinPeakProminence = 0.1;

left_index = 17;
right_index = 30;

%% 场景参数 这里假设是垂直场景
mirror_distance = -4;
target_loc = [1,0];
%% 角度估计卡尔曼滤波参数
Sigma_angle_move = 0.05;
Sigma_angle_ob = 0.05;






