function prob = BinarySensorProb(y,paramAgent,targetPos,paramSensor)

nAgent = length(y);

prob = 1;

for iAgent = 1:nAgent
    
    if y(iAgent) == 1 % when sensor does detect
        if IsInCircleBasedRegion(targetPos,paramAgent(iAgent).s,paramSensor.regionRadius)
            prob = prob*paramSensor.detectBeta;
        else
            prob = prob*0.01;
        end
    elseif y(iAgent) == 0 % when sensor does not detect
        if IsInCircleBasedRegion(targetPos,paramAgent(iAgent).s,paramSensor.regionRadius)
            prob = prob*(1-paramSensor.detectBeta);
        else
            prob = prob*0.99;
        end
    end 
    
end

end