function P = G_measure(x_nk,a_nk,Z_n)
    f_FA = 1;%/(pi * 15^2);
    global sigmaR;
    if a_nk == 0
        P = 1;
    else
        m = a_nk;
        Z_nm = Z_n{m};
        XY_measure = polar2rect(Z_nm(1),Z_nm(1,2));
        P = normpdf(XY_measure(1),x_nk(1),sigmaR) * normpdf(XY_measure(2),x_nk(2),sigmaR)/f_FA;
    end
end