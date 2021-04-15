% syms x;
% syms y;
% state = hx(x,y);
% J=jacobian(state,[x,y]);
% 
% function [state] = hx(x,y)
%     state(1,1) = norm([x,y]);
%     state(2,1) = atan2(y,x);
% end

function Hx = Hx_los(x,y)
    Hx(1,1) = (abs(x)*sign(x))/(abs(x)^2 + abs(y)^2)^(1/2);
    Hx(1,2) = (abs(y)*sign(y))/(abs(x)^2 + abs(y)^2)^(1/2);

    Hx(2,1) = -(imag(x) + real(y))/((imag(x) + real(y))^2 + (imag(y) - real(x))^2);
    Hx(2,2) = -(imag(y) - real(x))/((imag(x) + real(y))^2 + (imag(y) - real(x))^2);
    Hx(2,3) = 0;
    Hx(2,4) = 0;
    Hx(1,3) = 0;
    Hx(1,4) = 0;
end