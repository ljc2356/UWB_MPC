function P = Q_measure(x_kj,r_kj,a_kj,Z_k)
global Pd;
global sigmaR;
global mu_C;

    if r_kj == 1
        if a_kj == 0
            P = 1 - Pd;
        else
            Z_km = Z_k{a_kj};
            XY_measure = polar2rect(Z_km(1),Z_km(2));
            f_measure = normpdf(XY_measure(1),x_kj(1),sigmaR) * normpdf(XY_measure(2),x_kj(2),sigmaR);
            P = Pd/mu_C * f_measure;
        end
    elseif r_kj == 0
        if a_kj == 0
            P = 1;
        else
            P = 0;
        end
    end
end