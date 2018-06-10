function prob = BinarySensorProb(y,agentPos,targetPos,paramSensor)

if y == 1 % when sensor does detect
    if IsInCircleBasedRegion(targetPos,agentPos,paramSensor.regionRadius) 
        prob = paramSensor.detectBeta;
    else
        prob = 1-paramSensor.detectBeta;
    end
else % when sensor does not detect
    if IsInCircleBasedRegion(targetPos,agentPos,paramSensor.regionRadius) 
        prob = 0;
    else
        prob = 1;
    end
end

end