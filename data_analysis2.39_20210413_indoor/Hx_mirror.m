% syms x;
% syms y;
% syms h;
% syms theta;
% state = h_mirror_TDOA(x,y,h,theta);
% J=jacobian(state,[x,y,h,theta]);
% 
% function state = h_mirror_TDOA(x,y,h,theta)
%     [xm,ym] = mirror(x,y,h,theta);
%     state = [ norm([xm,ym] - norm([x,y]));
%            atan2(ym,xm)];
% end

function Hx = Hx_mirror(x,y,h,theta)
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

