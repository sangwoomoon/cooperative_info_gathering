function Q_total = GatherProcNoiseCovMatrix( obj )
%GATHERPROCCOVMATRIX generates process covariance matrix for SINGLE agent
%that matches to all targets

% assume that Q for targets are known by agents.. should be modified!


Q_total = [];

% for bias part : it depends on the spec of sensor
for iSensor = 1 : length(obj.SENSOR)
    if strcmp(obj.SENSOR{iSensor}.spec,'RelCartBias') == 1 % when bias is considered
        Q_total = blkdiag(Q_total,obj.SENSOR{iSensor}.Q);
    end
end

for iTarget = 1 : length(obj.TARGET)
    Q_total = blkdiag(Q_total,obj.TARGET(iTarget).DYNAMICS.Q); % same sensor to measure different targets
end

end

