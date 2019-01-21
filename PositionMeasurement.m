function meas = PositionMeasurement(targetState,sensorParam)

nState = length(targetState);
measNoise = mvnrnd(zeros(nState,1),sensorParam.R)';

% y = x + w
meas = targetState + measNoise;

end