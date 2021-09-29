function A = AOA_Phi(theta,fc,c,radius)
    %% 输出MUSIC算法所需的A矩阵
    for i = 1:8
        alpha(i) = wrapToPi((i-1)*pi/4);
    end
    A(1,1:length(theta)) = 1;
    for i = 2:8
        A(i,1:length(theta)) = exp(1j * (2 * pi * fc / c * radius * cos(theta - alpha(i))))./exp(1j * (2 * pi * fc / c * radius * cos(theta - alpha(1))));
    end
end