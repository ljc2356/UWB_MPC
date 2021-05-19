function [y] = sinc_interp(x, N, isplot)
%% sinc插值函数
% 输入
% x 待插值序列索引
% N 插值倍数
% isplot 是否画图
% 输出
% y 插值序列结果
%%
    if nargin == 2
        isplot = 0;
    end
    x_up_zero = [x, zeros(numel(x), N-1)];
    x_up_zero = reshape(x_up_zero.', N*numel(x), 1);
    x_up_zero = x_up_zero(1:end-N+1);
    sinc_x = -16*N:1/N:16*N;
    y = conv(x_up_zero, sinc(sinc_x), 'same');
    if isplot
        figure
        plot(1:numel(x), x, 'ro')
        hold on
        plot(1:1/N:numel(x), y, 'b-*')
    end
end

