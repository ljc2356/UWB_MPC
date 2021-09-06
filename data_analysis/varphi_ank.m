function result = varphi_ank(a_nk)
    global Pd;
    muFA = 0.2;
    if a_nk == 0
        result = 1 - Pd;
    else
        result = Pd/muFA;
    end
end