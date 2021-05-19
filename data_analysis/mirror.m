function [xm,ym] = mirror(x,y,h,theta)
%% 镜像点生成函数
% 输入
% x     标签x坐标
% y     标签y坐标
% h     反射面纵轴截距
% theta 反射面x轴倾角
% 输出
% xm    镜像点x坐标
% ym    镜像点y坐标
%%
    xm = x.*cos(2*theta) + (y - h).*sin(2*theta);
    ym = h + (h-y).*cos(2*theta) + x .* sin(2*theta);
end