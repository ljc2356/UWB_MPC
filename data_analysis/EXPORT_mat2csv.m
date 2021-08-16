clear all;close all;clc
load('data/20210413_indoor/move_07.mat');


result_mat = [result(6,1).los_d.data, result(6,1).los_phi.data, result(6,1).mpc_d.data , result(6,1).mpc_phi.data];
csvwrite("move_07.csv",result_mat);
