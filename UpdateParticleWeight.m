function wNext = UpdateParticleWeight(measNow,ptNow,paramAgent,paramSensor,property)

nPt = length(ptNow(1,:));
nAgent = length(measNow(1,:));

wNext = (1/nPt)*ones(1,nPt);

for iAgent = 1:nAgent
    if ~isnan(measNow(1,iAgent)) % when the emasurement exist
        
        switch property
            case 'PosLinear'
                prob = LinearSensorProb(measNow(:,iAgent),ptNow,paramSensor);
            case 'range_bear'
                prob = RangeBearProb(measNow(:,iAgent),paramAgent,ptNow,paramSensor);
            case 'detection'
                prob = BinarySensorProb(measNow(:,iAgent),paramAgent,ptNow,paramSensor);
        end
        
        
        if sum(wNext) == 0 % if all weights are zero
            wNext = wNext.*((1/nPt)*ones(1,nPt));
        else
            wNext = wNext.*prob;
        end
        
    end
    
    % if not (delivered measurement
    
end

% normalization
wNext = wNext./sum(wNext);
    
end