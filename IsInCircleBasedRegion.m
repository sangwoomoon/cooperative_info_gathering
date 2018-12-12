% sub-function for BinarySensorModel: Circle-based determinent
% element number of output discreminent = number of target state
function bLocate = IsInCircleBasedRegion(targetState,agentState,radius)

    diffX = targetState(1,:,:)-agentState(1);
    diffY = targetState(2,:,:)-agentState(2);
    
    dist = sqrt(diffX.^2 + diffY.^2);
    bLocate = dist < radius;

end