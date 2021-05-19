% clear all; clc;
Nfiles = 4;
Nuwb = 8;
folder = '../ml_data/20210413_indoor/';
filenames = cell(Nfiles, 1);

filenames{1} = [folder, 'cali_01.json'];
filenames{2} = [folder, 'cali_02.json'];
filenames{3} = [folder, 'cali_03.json'];
filenames{4} = [folder, 'cali_04.json'];
real_positions = [1, 0, 0
    0, 1, 0;
    -1, 0, 0;
    0, -1, 0
    ];
antArray = [0.0732,0,0;
         0.0518, 0.0518,0;
        0,0.0732,0;
        -0.0518, 0.0518,0;
        -0.0732,0,0;
        -0.0518, -0.0518,0;
        0,-0.0732,0;
        0.0518, -0.0518,0];
lambda = 3e8/(499.2e6*8); % Notice.
records = cell(Nfiles, 1);
vecData = struct();
for n = 1:Nfiles
    records{n} = loadRecordFile(filenames{n});
    [vecData(n, 1).Valid, vecData(n, 1).Reliability, vecData(n, 1).Pdoa,...
        vecData(n, 1).Tdoa, vecData(n, 1).D, vecData(n, 1).drsmpdel] = record2vec(records{n});
end
pdoa_correction = zeros(Nfiles, Nuwb);
tdoa_correction = zeros(Nfiles, Nuwb);
pdoa_std = zeros(Nfiles, Nuwb);
tdoa_std = zeros(Nfiles, Nuwb);
for n = 1:Nfiles
    distance = mean(vecData(n).D);
    position = real_positions(n, :) / norm(real_positions(n, :)) * distance;
    [pdoa, tdoa] = calcRealPdoaTdoa(antArray, position, lambda);
    for s = 2:Nuwb
        Mask = vecData(n).Valid(:, s);% & vecData(n).Reliability(:, s) > 0.5;
        pdoa_std(n, s) = std(vecData(n).Pdoa(Mask, s));
        tdoa_std(n, s) = std(vecData(n).Tdoa(Mask, s));
        pdoa_correction(n, s) = wrapToPi(mean(vecData(n).Pdoa(Mask, s)) - pdoa(s));
        tdoa_correction(n, s) = mean(vecData(n).Tdoa(Mask, s)) - tdoa(s);
    end
end
W = pdoa_std(:, 2:end).^-2;
P = pdoa_correction(:, 2:end);
T = tdoa_correction(:, 2:end);
pdoa_cali = zeros(1, Nuwb);
tdoa_cali = zeros(1, Nuwb);
for n = 2:Nuwb
    if sum(W(:, n-1)) ~= 0 && ~isnan(sum(W(:, n-1)))
        pdoa_cali(n) = wrapToPi(sum(W(:, n-1).*theta_postprocess(P(:, n-1)))/sum(W(:, n-1)));
        tdoa_cali(n) = sum(W(:, n-1).*T(:, n-1))/sum(W(:, n-1));
    else
        pdoa_cali(n) = 0;
        tdoa_cali(n) = 0;
    end
end
fid = fopen('DBB31430FF48C24.json');
data = [];
while ~feof(fid)
    data = [data, fgetl(fid)];
end
fclose(fid);
data = jsondecode(data);
data.phase_diff_cali = data.phase_diff_cali + pdoa_cali';
data.time_diff_cali = data.time_diff_cali + tdoa_cali';
jsonencode(data)
