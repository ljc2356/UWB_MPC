function allrecords = loadRecordFile(filename, isMeaResultAgg)
if nargin == 1
    isMeaResultAgg = false;
end
fid = fopen(filename);
index = 0;
while ~feof(fid)
    str = fgetl(fid);
    record = parseRecordFile(str, isMeaResultAgg);
    index = index + 1;
    allrecords(index, 1) = record;
end
fclose(fid);
if index >= 2
    allid = zeros(index, 1);
    for n = 1:index
        allid(n) = allrecords(n).id;
    end
    [~, I] = sort(allid, 'ascend');
    allrecords = allrecords(I);
end
end

function record = parseRecordFile(jsonstr, isMeaResultAgg)
recordOri = jsondecode(jsonstr);
record = struct();
record.id = recordOri.recordHeader.recordId;
if isfield(recordOri.recordHeader.dataTimestamp, 'secords')
    sec = str2double(recordOri.recordHeader.dataTimestamp.seconds);
else
    sec = 0;
end
if isfield(recordOri.recordHeader.dataTimestamp, 'nanos')
    record.time = ConvertDate(sec, recordOri.recordHeader.dataTimestamp.nanos);
else
    record.time = ConvertDate(sec, 0);
end
%R = recordOri.recordHeader.resultHeader.respCount;
R = 1;
C = recordOri.recordHeader.resultHeader.uwbCount;
record.R = R;
record.C = C;
record.channel = recordOri.recordHeader.resultHeader.channel;
record.cirStartOffset = recordOri.recordHeader.resultHeader.cirStartOffset;
record.V = false(R, C);
record.uwbResult = struct();
record.uwbResult.rxts = uint64(zeros(R, C));
record.uwbResult.rsmpdel = zeros(R, C);
record.uwbResult.cir = cell(R, C);
for r = 1:R
    if isstruct(recordOri.uwbResult.respResults(r).singleUwbResults)
        tmp = cell(C, 1);
        for c = 1:C
            tmp{c} = recordOri.uwbResult.respResults(r).singleUwbResults(c);
        end
        recordOri.uwbResult.respResults(r).singleUwbResults = tmp;
    end
    for c = 1:C
        if bitget(recordOri.uwbResult.respResults(r).recvUwbIndex, c) == 1
            record.V(r, c) = true;
            record.uwbResult.rxts(r, c) = uint64(str2double(recordOri.uwbResult.respResults(r).singleUwbResults{c}.respRxTs));
            if isfield(recordOri.uwbResult.respResults(r).singleUwbResults{c}, 'respRsmpdel')
                record.uwbResult.rsmpdel(r, c) = recordOri.uwbResult.respResults(r).singleUwbResults{c}.respRsmpdel;
            else
                record.uwbResult.rsmpdel(r, c) = 0;
            end
            record.uwbResult.cir{r, c} = recordOri.uwbResult.respResults(r).singleUwbResults{c}.respCir.cirReals + 1j * recordOri.uwbResult.respResults(r).singleUwbResults{c}.respCir.cirImags;
        end
    end
end
record.meaResult = struct();
record.meaResult.D = recordOri.meaResult.distance;
if isMeaResultAgg
    record.meaResult.reliability = zeros(1, C);
    record.meaResult.pdoa = zeros(1, C);
    record.meaResult.tdoa = zeros(1, C);
    r = 1;
    for c = 1:C
        if record.V(r, c)
            if isfield(recordOri.meaResult.respMeaResults(r).singleMeaResults(c), 'reliability')
                record.meaResult.reliability(r, c) = recordOri.meaResult.respMeaResults(r).singleMeaResults(c).reliability;
            else
                record.meaResult.reliability(r, c) = 0;
            end
            if c ~= 1
                record.meaResult.pdoa(r, c) = recordOri.meaResult.respMeaResults(r).singleMeaResults(c).phaseDiff;
                if isfield(recordOri.meaResult.respMeaResults(r).singleMeaResults(c), 'timeDiff')
                    record.meaResult.tdoa(r, c) = recordOri.meaResult.respMeaResults(r).singleMeaResults(c).timeDiff;
                else
                    record.meaResult.tdoa(r, c) = 0;
                end
            end
        end
    end
else
    record.meaResult.reliability = zeros(R, C);
    record.meaResult.pdoa = zeros(R, C);
    record.meaResult.tdoa = zeros(R, C);
    for r = 1:R
        for c = 1:C
            if record.V(r, c)
                if isfield(recordOri.meaResult.respMeaResults(r).singleMeaResults(c), 'reliability')
                    record.meaResult.reliability(r, c) = recordOri.meaResult.respMeaResults(r).singleMeaResults(c).reliability;
                else
                    record.meaResult.reliability(r, c) = 0;
                end
                if c ~= 1
                    record.meaResult.pdoa(r, c) = recordOri.meaResult.respMeaResults(r).singleMeaResults(c).phaseDiff;
                    if isfield(recordOri.meaResult.respMeaResults(r).singleMeaResults(c), 'timeDiff')
                        record.meaResult.tdoa(r, c) = recordOri.meaResult.respMeaResults(r).singleMeaResults(c).timeDiff;
                    else
                        record.meaResult.tdoa(r, c) = 0;
                    end
                end
            end
        end
    end
end
record.mlResult = struct();
if isfield(recordOri, 'mlResult') && isfield(recordOri.mlResult, 'mlProbs')
    N = numel(recordOri.mlResult.mlProbs);
    if N ~= 0
        record.mlResult.weight = zeros(N, 1);
        record.mlResult.position = zeros(N, 3);
        record.mlResult.sigma = cell(N, 1);
        for n = 1:N
            record.mlResult.position(n, 1) = recordOri.mlResult.mlProbs(n).x;
            record.mlResult.position(n, 2) = recordOri.mlResult.mlProbs(n).y;
            record.mlResult.position(n, 3) = recordOri.mlResult.mlProbs(n).z;
            record.mlResult.weight(n) = recordOri.mlResult.mlProbs(n).w;
            record.mlResult.sigma{n} = reshape(recordOri.mlResult.mlProbs(n).locSigmaMat, 3, 3);
        end
    end
end
record.locResult = struct();
if isfield(recordOri, 'gmmkfLocResult')
    N = numel(recordOri.gmmkfLocResult.locProbs);
    if N ~= 0
        record.locResult.weight = zeros(N, 1);
        record.locResult.position = zeros(N, 3);
        record.locResult.sigma = cell(N, 1);
        for n = 1:N
            record.locResult.position(n, 1) = recordOri.gmmkfLocResult.locProbs(n).x;
            record.locResult.position(n, 2) = recordOri.gmmkfLocResult.locProbs(n).y;
            record.locResult.position(n, 3) = recordOri.gmmkfLocResult.locProbs(n).z;
            record.locResult.weight(n) = recordOri.gmmkfLocResult.locProbs(n).w;
            record.locResult.sigma{n} = reshape(recordOri.gmmkfLocResult.locProbs(n).locSigmaMat, 3, 3);
        end
    end
end
end
