function [xm,ym] = mirror(x,y,h,theta)
    xm = x.*cos(2*theta) + (y - h).*sin(2*theta);
    ym = h + (h-y).*cos(2*theta) + x .* sin(2*theta);
end