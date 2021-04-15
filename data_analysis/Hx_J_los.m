
function Hx = Hx_J_los(m)

    x = m(1,1);
    y = m(2,1);
    Hx(1,1) = (abs(x)*sign(x))/(abs(x)^2 + abs(y)^2)^(1/2);
    Hx(1,2) = (abs(y)*sign(y))/(abs(x)^2 + abs(y)^2)^(1/2);

    Hx(2,1) = -(imag(x) + real(y))/((imag(x) + real(y))^2 + (imag(y) - real(x))^2);
    Hx(2,2) = -(imag(y) - real(x))/((imag(x) + real(y))^2 + (imag(y) - real(x))^2);
    Hx(2,3) = 0;
    Hx(2,4) = 0;
    Hx(1,3) = 0;
    Hx(1,4) = 0;
end