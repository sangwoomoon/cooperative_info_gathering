function Y = GatherMeasurements( obj )
%GATHERMEASUREMENTS gathers measurements for all targets to use in
%Estimator class

Y = [];
for iMeasure = 1 : length(obj.meas)
    Y = [Y;obj.meas(iMeasure).y];
end


end

