function Hx = Hx_J_mirror(m)
%% 多径观测雅各比矩阵生成
% 输入 m 标签参数列表
%m 参数列表 
% m(1,1) 标签x坐标
% m(2,1) 标签y坐标
% m(3,1) 标签x速度
% m(4,1) 标签y速度
% m(5,1) 反射面y轴截距
% m(6,1) 反射面相对x轴倾角

% 输出 Hx 多径观测雅各比矩阵

%注意 由于观测中使用atan2函数 并非atan 因此对atan函数的求导的结果不适合此处 这里使用的雅各比矩阵为MATLAB 自动生成
%%
    x = m(1,1);
    y = m(2,1);
    h = m(5,1);
    theta = m(6,1);
    Hx(1,1) = (2*sin(2*theta)*abs(h + x*sin(2*theta) + cos(2*theta)*(h - y))*sign(h + x*sin(2*theta) + cos(2*theta)*(h - y)) + 2*abs(x*cos(2*theta) - sin(2*theta)*(h - y))*sign(x*cos(2*theta) - sin(2*theta)*(h - y))*cos(2*theta))/(2*(abs(h + x*sin(2*theta) + cos(2*theta)*(h - y))^2 + abs(x*cos(2*theta) - sin(2*theta)*(h - y))^2)^(1/2));
    Hx(1,2) = -(2*cos(2*theta)*abs(h + x*sin(2*theta) + cos(2*theta)*(h - y))*sign(h + x*sin(2*theta) + cos(2*theta)*(h - y)) - 2*abs(x*cos(2*theta) - sin(2*theta)*(h - y))*sign(x*cos(2*theta) - sin(2*theta)*(h - y))*sin(2*theta))/(2*(abs(h + x*sin(2*theta) + cos(2*theta)*(h - y))^2 + abs(x*cos(2*theta) - sin(2*theta)*(h - y))^2)^(1/2));
    Hx(1,5) = (2*abs(h + x*sin(2*theta) + cos(2*theta)*(h - y))*sign(h + x*sin(2*theta) + cos(2*theta)*(h - y))*(cos(2*theta) + 1) - 2*abs(x*cos(2*theta) - sin(2*theta)*(h - y))*sign(x*cos(2*theta) - sin(2*theta)*(h - y))*sin(2*theta))/(2*(abs(h + x*sin(2*theta) + cos(2*theta)*(h - y))^2 + abs(x*cos(2*theta) - sin(2*theta)*(h - y))^2)^(1/2));
    Hx(1,6) = (2*abs(h + x*sin(2*theta) + cos(2*theta)*(h - y))*sign(h + x*sin(2*theta) + cos(2*theta)*(h - y))*(2*x*cos(2*theta) - 2*sin(2*theta)*(h - y)) - 2*abs(x*cos(2*theta) - sin(2*theta)*(h - y))*sign(x*cos(2*theta) - sin(2*theta)*(h - y))*(2*x*sin(2*theta) + 2*cos(2*theta)*(h - y)))/(2*(abs(h + x*sin(2*theta) + cos(2*theta)*(h - y))^2 + abs(x*cos(2*theta) - sin(2*theta)*(h - y))^2)^(1/2));
    
    Hx(2,1) = -(((imag(cos(2*theta)) + real(sin(2*theta)))/(imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h)) + ((real(cos(2*theta)) - imag(sin(2*theta)))*(imag(x*cos(2*theta)) + real(x*sin(2*theta)) + real(cos(2*theta)*(h - y)) - imag(sin(2*theta)*(h - y)) + real(h)))/(- real(x*cos(2*theta)) + imag(x*sin(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2)*(imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2)/((imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2 + (imag(x*cos(2*theta)) + real(x*sin(2*theta)) + real(cos(2*theta)*(h - y)) - imag(sin(2*theta)*(h - y)) + real(h))^2);
    Hx(2,2) = (((real(cos(2*theta)) - imag(sin(2*theta)))/(imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h)) - ((imag(cos(2*theta)) + real(sin(2*theta)))*(imag(x*cos(2*theta)) + real(x*sin(2*theta)) + real(cos(2*theta)*(h - y)) - imag(sin(2*theta)*(h - y)) + real(h)))/(- real(x*cos(2*theta)) + imag(x*sin(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2)*(imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2)/((imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2 + (imag(x*cos(2*theta)) + real(x*sin(2*theta)) + real(cos(2*theta)*(h - y)) - imag(sin(2*theta)*(h - y)) + real(h))^2);
    Hx(2,5) = -(((real(cos(2*theta)) - imag(sin(2*theta)) + 1)/(imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h)) - ((imag(cos(2*theta)) + real(sin(2*theta)))*(imag(x*cos(2*theta)) + real(x*sin(2*theta)) + real(cos(2*theta)*(h - y)) - imag(sin(2*theta)*(h - y)) + real(h)))/(- real(x*cos(2*theta)) + imag(x*sin(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2)*(imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2)/((imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2 + (imag(x*cos(2*theta)) + real(x*sin(2*theta)) + real(cos(2*theta)*(h - y)) - imag(sin(2*theta)*(h - y)) + real(h))^2);
    Hx(2,6) = (((2*imag(x*sin(2*theta)) - 2*real(x*cos(2*theta)) + 2*imag(cos(2*theta)*(h - y)) + 2*real(sin(2*theta)*(h - y)))/(imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h)) + ((2*imag(x*cos(2*theta)) + 2*real(x*sin(2*theta)) + 2*real(cos(2*theta)*(h - y)) - 2*imag(sin(2*theta)*(h - y)))*(imag(x*cos(2*theta)) + real(x*sin(2*theta)) + real(cos(2*theta)*(h - y)) - imag(sin(2*theta)*(h - y)) + real(h)))/(- real(x*cos(2*theta)) + imag(x*sin(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2)*(imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2)/((imag(x*sin(2*theta)) - real(x*cos(2*theta)) + imag(cos(2*theta)*(h - y)) + real(sin(2*theta)*(h - y)) + imag(h))^2 + (imag(x*cos(2*theta)) + real(x*sin(2*theta)) + real(cos(2*theta)*(h - y)) - imag(sin(2*theta)*(h - y)) + real(h))^2);

    Hx(1,3) = 0;
    Hx(1,4) = 0;
    Hx(2,3) = 0;
    Hx(2,4) = 0;
    
end

