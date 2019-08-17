function meas = BearMeasurement(targetState,agentState,sensorParam)

bear = ComputeBear(targetState,agentState);

meas = bear + mvnrnd(0,sensorParam.R)';

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