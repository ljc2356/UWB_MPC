%% parameter configuration list reused among various sub-functions

%% Calibration data (if you want to modify it, you need to modify it jointly with the calibration json)
cali = wrapToPi([0,-3.8786237648942614,-0.1800383535470497,-3.8974087631917986,-2.3507412761762176,2.084888347044676,6.7786643525842818,0.17383465169886048]);

%% PDOA angle measurement Antenna calibration parameters
fc = 3.9936e+9;
c = 299792458;
radius = 0.0732;


formation{3} = [1 2 3];
formation{4} = [1 2 3 4];
formation{5} = [1 2 3 4 5];
formation{6} = [1 2 3 4 5 6];
formation{7} = [1 2 3 4 5 6 7 ];
formation{8} = [1 2 3 4 5 6 7 8];

%% Multipath extraction parameters

los_path = 8; %The first path is 8 by default

window_width =9;% Note that the length of the window is required to be an odd number here
first_start_window = 10-(window_width-1)/2;
center_index = 10;

intersize = 64;
MinPeakProminence = 1;

%% Scene parameters This assumes that it is a vertical scene
mirror_distance = -4;
target_loc = [1,0];
%% angle estimation Kalman filter parameters
Sigma_angle_move = 0.05;
Sigma_angle_ob = 0.05;






