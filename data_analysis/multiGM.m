function [mu3,sigma3,coeff3] = multiGM(mu1,sigma1,coeff1,mu2,sigma2,coeff2)
    mu3 = (mu1*sigma2^2 + mu2 * sigma1^2)/(sigma1^2 + sigma2^2);
    sigma3 = sqrt((sigma1^2)*(sigma2^2)/(sigma1^2 + sigma2^2));
    coeff3 = coeff1 * coeff2 * 1 / sqrt(2*pi*(sigma1^2 + sigma2^2)) *...
                    exp(-1 * ((norm(mu1 - mu2))^2)/( 2 * (sigma1^2 + sigma2^2))) + 0.0000000000001;
end