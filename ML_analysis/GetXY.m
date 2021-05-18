function XY = GetXY(filename)
    x = str2num(filename(1));
    if filename(2) == '-'
        y = -1 * str2num(filename(3));
    else
        y = str2num(filename(2));
    end
    XY = [x,y];
end