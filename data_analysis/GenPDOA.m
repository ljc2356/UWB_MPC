function PDOA = GenPDOA(theta)
    
    run("Properties.m");
    for i = 1:8
        alpha(i) = wrapToPi((i-1)*pi/4);
    end
    
    std_phi = 2 * pi * fc / c * radius * cos(theta);
    
    for i = 1:8
        phi(i,:) =    2 * pi * fc / c * radius * cos(theta - alpha(i));
        predict_pdoa(i,:) = wrapToPi( phi(i,:) - std_phi );
    end
    PDOA = predict_pdoa';
end