function result = varphi_ank(a_nk)
    Pd = 0.85;
    muFA = 1;
    if a_nk == 0
        result = 1 - Pd;
    else
        result = Pd/muFA;
    end
end