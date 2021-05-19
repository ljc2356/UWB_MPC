% syms x;
% syms y;
% state = hx(x,y);
% J=jacobian(state,[x,y]);
% 
% function [state] = hx(x,y)
%     state(1,1) = norm([x,y]);
%     state(2,1) = atan2(y,x);
% end

function Hx = Hx_J_los(m)
%% 首达径观测雅各比矩阵生成
% 输入 m 标签参数列表
%m 参数列表 
% m(1,1) 标签x坐标
% m(2,1) 标签y坐标
% m(3,1) 标签x速度
% m(4,1) 标签y速度
% m(5,1) 反射面y轴截距
% m(6,1) 反射面相对x轴倾角

% 输出 Hx 直达径观测雅各比矩阵

%注意 由于观测中使用atan2函数 并非atan 因此对atan函数的求导的结果不适合此处 这里使用的雅各比矩阵为MATLAB 自动生成
%生成代码在函数上方注释中
%%
    x = m(1,1);
    y = m(2,1);
    Hx(1,1) = (abs(x)*sign(x))/(abs(x)^2 + abs(y)^2)^(1/2);
    Hx(1,2) = (abs(y)*sign(y))/(abs(x)^2 + abs(y)^2)^(1/2);

    Hx(2,1) = -(imag(x) + real(y))/((imag(x) + real(y))^2 + (imag(y) - real(x))^2);
    Hx(2,2) = -(imag(y) - real(x))/((imag(x) + real(y))^2 + (imag(y) - real(x))^2);
    Hx(2,3) = 0;
    Hx(2,4) = 0;
    Hx(1,3) = 0;
    Hx(1,4) = 0;
end