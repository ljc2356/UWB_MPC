% syms x;
% syms y;
% state = hx(x,y);
% J=jacobian(state,[x,y]);
%
% function [state] = hx(x,y)
% state(1,1) = norm([x,y]);
% state(2,1) = atan2(y,x);
% end

function Hx = Hx_J_los(m)
%% Jacobian matrix generation for first-reaching trail observation
% Enter the m tag parameter list
%m parameter list
% m(1,1) label x coordinate
% m(2,1) label y coordinate
% m(3,1) label x speed
% m(4,1) label y speed
% m(5,1) y-axis intercept of the reflecting surface
% m(6,1) The inclination angle of the reflecting surface relative to the x-axis

% Output Hx direct path observation Jacobian matrix

%Note Since the atan2 function used in the observation is not atan, the result of the derivative of the atan function is not suitable here. The Jacobian matrix used here is automatically generated by MATLAB.
% The generated code is in the comment above the function
%%
    x = m(1,1);
    y = m(2,1);
    Hx(1,1) = (abs(x)*sign(x))/(abs(x)^2 + abs(y)^2)^(1/2);
    Hx(1,2) = (abs(y)*sign(y))/(abs(x)^2 + abs(y)^2)^(1/2);

    Hx(2,1) = -(imag(x) + real(y))/((imag(x) + real(y))^2 + (imag(y)-real(x))^2);
    Hx(2,2) = -(imag(y)-real(x))/((imag(x) + real(y))^2 + (imag(y)-real(x))^2);
    Hx(2,3) = 0;
    Hx(2,4) = 0;
    Hx(1,3) = 0;
    Hx(1,4) = 0;
end