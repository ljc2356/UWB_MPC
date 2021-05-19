function [pdoa, tdoa] = calcRealPdoaTdoa(antArray, position, lambda, wrapPdoa)
if nargin == 3
    wrapPdoa = 1;
end
N = size(antArray, 1);
distance = zeros(N, 1);
for n = 1:N
    distance(n) = norm(antArray(n, :) - position);
end
if wrapPdoa == 1
    pdoa = wrapToPi(-(distance - distance(1)) / lambda * 2 * pi);
else
    pdoa = -(distance - distance(1)) / lambda * 2 * pi;
end
tdoa = (distance - distance(1)) / 3e8 * (499.2e6*128);
end

