function wNext = UpdateParticleWeight(measNow,ptNow,paramAgent,paramSensor)

nPt = length(ptNow(1,:));

wNext = BinarySensorProb(measNow,paramAgent,ptNow,paramSensor);


if sum(wNext) == 0 % if all weights are zero
    wNext = (1/nPt)*ones(1,nPt);
else
    wNext = wNext./sum(wNext);
end
    
end