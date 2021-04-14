function ob = h_mirror(m)
    [xm,ym] = mirror(m(1,1),m(2,1),m(5,1),m(6,1));
    ob = [ norm([xm,ym]);
           atan2(ym,xm)];
end