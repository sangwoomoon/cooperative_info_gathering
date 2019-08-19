function meas = RFMeasurement(targetState,agentState,sensorParam)

rss = ComputeRSS(targetState,agentState);

meas = rss + mvnrnd(0,sensorParam.R)';

end


function rss = ComputeRSS(targetState,agentState)

nState = length(targetState(:,1));

switch nState
    case 4 % 2D static with RF info
        
        dist = sqrt((targetState(1,:)-agentState(1)).^2 + (targetState(2,:)-agentState(2)).^2);
        rss = 10.*log10(targetState(3,:)./dist.^targetState(4,:));
        
    case 5 % 3D static with RF info 
        
    case 6 % 2D moving with RF info
        
    case 8 % 3D moving with RF info
        
end

end