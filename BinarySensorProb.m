function prob = BinarySensorProb(y,paramAgent,targetPos,paramSensor)

nAgent = length(y);

% % check whether particle is in the sensing region of any of agents
% bPtInside = 0;
% for iAgent = 1:nAgent
%     if IsInCircleBasedRegion(targetPos,paramAgent(iAgent).s,paramSensor.regionRadius)
%         bPtInside = 1;
%     end
% end

prob = 1;

for iAgent = 1:nAgent
    
    if y(iAgent) == 1 % when sensor does detect
        if IsInCircleBasedRegion(targetPos,paramAgent(iAgent).s,paramSensor.regionRadius)
            prob = prob*paramSensor.detectBeta;
        else
            prob = prob*(1-paramSensor.detectBeta);
        end
    else % when sensor does not detect
        if IsInCircleBasedRegion(targetPos,paramAgent(iAgent).s,paramSensor.regionRadius)
            prob = prob*0;
        else
            prob = prob*1;
        end
    end
    
end

end