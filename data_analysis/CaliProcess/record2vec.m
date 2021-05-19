function [Valid, Reliability, Pdoa, Tdoa, D, drsmpdel] = record2vec(allrecords)
N = numel(allrecords);
[resp, uwb] = size(allrecords(1).V);
nmea = size(allrecords(1).meaResult.pdoa, 1);
Valid = false(N*nmea, uwb);
Reliability = zeros(N*nmea, uwb);
Pdoa = zeros(N*nmea, uwb);
Tdoa = zeros(N*nmea, uwb);
D = zeros(N, 1);
drsmpdel = zeros(N*resp, uwb);
for n = 1:N
    if nmea == resp
        Valid(nmea*n-(nmea-1):nmea*n, :) = allrecords(n).V;
    else
        Valid(n, :) = (sum(allrecords(n).V) ~= 0);
    end
    Reliability(nmea*n-(nmea-1):nmea*n, :) = allrecords(n).meaResult.reliability;
    Pdoa(nmea*n-(nmea-1):nmea*n, :) = allrecords(n).meaResult.pdoa;
    Tdoa(nmea*n-(nmea-1):nmea*n, :) = allrecords(n).meaResult.tdoa;
%     Tdoa(resp*n-(resp-1):resp*n, :) = double(allrecords(n).uwbResult.rxts) - double(allrecords(n).uwbResult.rxts(:, 1)) * ones(1, 8);
    D(n) = allrecords(n).meaResult.D;
    tmp = allrecords(n).uwbResult.rsmpdel;
    drsmpdel(resp*n-(resp-1):resp*n, 2:uwb) = tmp(:, 2:uwb) - tmp(:, 1) * ones(1, uwb-1);
end
for c = 1:uwb
    Pdoa(Valid(:, c), c) = theta_postprocess(Pdoa(Valid(:, c), c));
end
drsmpdel(drsmpdel > 32) = drsmpdel(drsmpdel > 32) - 64;
drsmpdel(drsmpdel < -32) = drsmpdel(drsmpdel < -32) + 64;
end

