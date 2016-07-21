function R_total = GatherMeasNoiseCovMatrix( obj )
%TAKEMEASCOVMATRIX generates measurement covariance matrix for SINGLE agent
%that matches to all targets

R_total = [];

% for bias part : it depends on the spec of sensor
if strcmp(obj.spec,'RelCartBias') == 1 % when bias is considered
    R_total = blkdiag(R_total,obj.R_bias);
end

for iMeasure = 1 : length(obj.meas)
    if length(obj.meas(iMeasure).id) == 1 % when it is not landmark
        R_total = blkdiag(R_total,obj.R); % same sensor to measure different targets
    end
end

end

