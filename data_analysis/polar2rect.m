function XY = polar2rect(d,theta)
    XY(1,1) = d * cos(theta);
    XY(2,1) = d * sin(theta);
end