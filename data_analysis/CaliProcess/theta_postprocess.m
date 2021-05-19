%% Smooth angles
function theta_post = theta_postprocess(theta, v)
if nargin == 1
    v = true(size(theta));
end
if numel(theta) == 0
    theta_post = [];
    return;
end
theta = wrapToPi(theta);
theta_post = zeros(size(theta));
nstart = 1;
while ~v(nstart)
    theta_post(nstart) = 0;
    nstart = nstart + 1;
end
theta_post(nstart) = theta(nstart);
offset = 0;
lastTheta = theta_post(nstart);
for n = nstart+1:numel(theta)
    if ~v(n)
        theta_post(n) = 0;
        continue
    end
    if theta(n) + offset - lastTheta > pi
        offset = offset - 2*pi;
    elseif theta(n) + offset - lastTheta < -pi
        offset = offset + 2*pi;
    end
    theta_post(n) = theta(n) + offset;
    lastTheta = theta_post(n);
end
end