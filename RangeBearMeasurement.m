function meas = RangeBearMeasurement(targetState,agentState,sensorParam)

range = ComputeRange(targetState,agentState);
bear = ComputeBear(targetState,agentState);

meas = [range; bear] + mvnrnd(zeros(2,1),sensorParam.R)';

end


function range = ComputeRange(targetState,agentState)

nState = length(targetState(:,1));

switch nState
    case 2 % 2D static
        range = sqrt((targetState(1,:)-agentState(1)).^2 + (targetState(2,:)-agentState(2)).^2);
    case 3 % 3D static
        
    case 4 % 2D moving
        
    case 6 % 3D moving

end

end

function bear = ComputeBear(targetState,agentState)

nState = length(targetState(:,1));

switch nState
    case 2 % 2D static
        bear = atan2(targetState(2,:)-agentState(2),targetState(1,:)-agentState(1));
    case 3 % 3D static
        
    case 4 % 2D moving
        
    case 6 % 3D moving
        
end

end