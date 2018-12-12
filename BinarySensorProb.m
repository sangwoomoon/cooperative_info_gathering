function prob = BinarySensorProb(y,paramAgent,targetPos,paramSensor)

nAgent = length(y);

nRefPt = size(targetPos);
prob = ones(1,nRefPt(2));

for iAgent = 1:nAgent
    
    if y(iAgent) == 1 % when sensor does detect
        bLocate = IsInCircleBasedRegion(targetPos,paramAgent(iAgent).s,paramSensor.regionRadius);
        prob = prob.*(bLocate*paramSensor.detectBeta + (1-bLocate)*(1-paramSensor.detectBeta));
    elseif y(iAgent) == 0 % when sensor does not detect
        bLocate = IsInCircleBasedRegion(targetPos,paramAgent(iAgent).s,paramSensor.regionRadius);
        prob = prob.*(bLocate*0.001 + (1-bLocate)*0.999);
    end
    
end
    

end