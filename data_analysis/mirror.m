function [xm,ym] = mirror(x,y,h,theta)
%% Mirror point generation function
% Input
% x label x coordinate
% y label y coordinate
% h Longitudinal intercept of reflecting surface
% theta x-axis inclination of the reflecting surface
% Output
% xm mirror point x coordinate
% ym mirror point y coordinate
%%
    xm = x.*cos(2*theta) + (y-h).*sin(2*theta);
    ym = h + (h-y).*cos(2*theta) + x .* sin(2*theta);
end