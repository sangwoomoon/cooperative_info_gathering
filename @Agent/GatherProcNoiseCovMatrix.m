function Q_total = GatherProcNoiseCovMatrix( obj, TARGET )
%GATHERPROCCOVMATRIX generates process covariance matrix for SINGLE agent
%that matches to all targets

% assume that Q for targets are known by agents.. should be modified!


Q_total = [];

% for bias part : it depends on the spec of sensor
if strcmp(obj.SENSOR.spec,'RelCartBias') == 1 % when bias is considered
    Q_total = blkdiag(Q_total,obj.SENSOR.R_bias);
end

for iTarget = 1 : length(TARGET)
    Q_total = blkdiag(Q_total,TARGET(iTarget).DYNAMICS.Q); % same sensor to measure different targets
end

end

