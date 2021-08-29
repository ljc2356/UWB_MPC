function XY = polar2rect(d,theta)
    XY(1) = d * cos(theta);
    XY(2) = d * sin(theta);
end