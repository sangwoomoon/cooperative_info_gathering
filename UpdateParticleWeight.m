function wNext = UpdateParticleWeight(measNow,ptNow,agentPos,paramSensor)

nPt = length(ptNow(1,:));
wNext = zeros(1,nPt);

for iPt = 1:nPt
    wNext(iPt) = BinarySensorProb(measNow,agentPos,ptNow(:,iPt),paramSensor);
end

if sum(wNext) == 0 % if all weights are zero
    wNext = (1/nPt)*ones(1,nPt);
else
    wNext = wNext./sum(wNext);
end
    
end