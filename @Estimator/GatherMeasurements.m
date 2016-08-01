function Y = GatherMeasurements( obj, SENSOR )
%GATHERMEASUREMENTS gathers measurements for all targets to use in
%Estimator class

Y = [];
for iSensor = 1 : length(SENSOR)
    for iMeasure = 1 : length(SENSOR{iSensor}.meas)
        Y = [Y;SENSOR{iSensor}.meas(iMeasure).y];
    end
end

end

