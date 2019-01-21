function wNext = UpdateParticleWeight(measNow,ptNow,paramAgent,paramSensor,property)

nPt = length(ptNow(1,:));

switch property
    case 'PosLinear'
        wNext = LinearSensorProb(measNow,ptNow,paramSensor);
    case 'range_bear'
    
    case 'detection'
        wNext = BinarySensorProb(measNow,paramAgent,ptNow,paramSensor);
end


if sum(wNext) == 0 % if all weights are zero
    wNext = (1/nPt)*ones(1,nPt);
else
    wNext = wNext./sum(wNext);
end
    
end