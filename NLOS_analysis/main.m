clear all;clc;close all;
%% 
run("Properties.m");
load("After_AnlyResult.mat")

Base_loc = [0,0];
antenna_num = 8;
lambda_0 = c/fc;
lambda_r = c/sqrt(epsilon)/fc;

for k = 1:antenna_num
    angle(k) = wrapToPi((k-1) * 2 * pi / antenna_num);
    antenna_loc(k,1) = Base_loc(1) + radius * cos(angle(k));
    antenna_loc(k,2) = Base_loc(2) + radius * sin(angle(k));
end

for i = 1:15
    x = str2num(AnlyResult(i,1).name(1));
    if AnlyResult(i,1).name(2) == '-'
        y = -1 * str2num(AnlyResult(i,1).name(3));
    else
        y = str2num(AnlyResult(i,1).name(2));
    end
    Target_loc = [(x+1)*0.6,y*0.6];
    %%
    for k = 1:antenna_num
        Theta_antenna_los(k) = atan2( Target_loc(2) - antenna_loc(k,2) , Target_loc(1) - antenna_loc(k,1));
        length_wall_antenna_los(k) = Width_wall / cos(Theta_antenna_los(k));
        num_lambda_air(k) = length_wall_antenna_los(k) / lambda_0;
        phi_air(k)  = num_lambda_air(k) * 2 * pi;
        num_lambda_wall(k) = length_wall_antenna_los(k) / lambda_r;
        phi_wall(k) = num_lambda_wall(k) * 2 * pi;
        pdoa_air(k) = phi_air(k) - phi_air(1);
        pdoa_wall(k) = phi_wall(k) - phi_wall(1);
        Diff_wall(k) = pdoa_wall(k) - pdoa_air(k);
    end
%% 
    pre_pdoa = wrapToPi(AnlyResult(i,1).realPdoa + Diff_wall);
    pre_angle = AOA_ML_Mat( pre_pdoa,formation{8},fc,c,radius,-pi,pi);
    AnlyResult(i,1).pre_angle = pre_angle;
    AnlyResult(i,1).pre_error = wrapToPi(AnlyResult(i,1).angle  - AnlyResult(i,1).pre_angle);

end