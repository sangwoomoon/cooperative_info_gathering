function R_total = GatherMeasNoiseCovMatrix( obj, Y_k )
%TAKEMEASCOVMATRIX generates measurement covariance matrix for SINGLE agent
%that matches to all targets

R_total = [];

% for bias part : it depends on the spec of sensor
for iSensor = 1 : length(obj.SENSOR)
    if strcmp(obj.SENSOR{iSensor}.spec,'RelCartBias') == 1 % when bias is considered
        R_total = blkdiag(R_total,obj.SENSOR{iSensor}.R);
    end

    for iMeasure = 1 : length(Y_k{iSensor})
        if length(Y_k{iSensor}(iMeasure).id) == 1 % when it is not landmark
            R_total = blkdiag(R_total,obj.SENSOR{iSensor}.R); % same sensor to measure different targets
        end
    end
end

end

