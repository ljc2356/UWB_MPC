clear all; clc; close all;
%%
load('move_03.mat')
figure()
fs_time = 0.05;
time = 0:fs_time:length(result(6,1).los_d.data)*fs_time;
hd = plot(time(1,1:end-1),result(6,1).los_phi.data)
xlabel('Time [s]');
ylabel('Angle [rad]');
set(gca,'FontSize',14);  
grid on;
set(hd,'color','b','linewidth',1);