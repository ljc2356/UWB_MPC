function ob = h_mirror(m)
%% Multipath observation model
% Input
%m parameter list
% m(1,1) label x coordinate
% m(2,1) label y coordinate
% m(3,1) label x speed
% m(4,1) label y speed
% m(5,1) y-axis intercept of the reflecting surface
% m(6,1) The inclination angle of the reflecting surface relative to the x-axis
% Output
%ob parameter vector
% ob(1,1) Virtual base station distance
% ob(2,1) Virtual base station angle
%%
    [xm,ym] = mirror(m(1,1),m(2,1),m(5,1),m(6,1));% Solve the label mirror position
    % Make observations
    ob = [norm([xm,ym]);
           atan2(ym,xm)];
end